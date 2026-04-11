from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any
from uuid import uuid4

import cv2
from cv_bridge import CvBridge
import rclpy
from brain_interfaces.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from sensor_msgs.msg import Image

from brain_nodes.constants import TOPIC_TRACKED_OBJECTS

try:
    from ultralytics import YOLO
except Exception:  # noqa: BLE001
    YOLO = None


@dataclass
class TrackState:
    track_id: str
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    estimated_distance_m: float
    last_seen_ns: int


class ObjectTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("object_tracker")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("detection_stride", 3)
        self.declare_parameter("track_ttl_sec", 1.0)
        self.declare_parameter("camera_vertical_fov_deg", 48.8)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.detection_stride = int(self.get_parameter("detection_stride").value)
        self.track_ttl_sec = float(self.get_parameter("track_ttl_sec").value)
        self.camera_vertical_fov_deg = float(self.get_parameter("camera_vertical_fov_deg").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(TrackedObjectArray, TOPIC_TRACKED_OBJECTS, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        self.model = None
        if YOLO is not None:
            self.model = YOLO(str(self.get_parameter("model_name").value))
        else:
            self.get_logger().warning("Ultralytics is unavailable; tracker will publish empty detections.")

        self.class_heights = {
            "person": 1.7,
            "chair": 0.9,
            "bottle": 0.25,
            "cup": 0.1,
            "book": 0.24,
            "tv": 0.6,
            "potted plant": 0.7,
        }
        self.tracks: dict[str, TrackState] = {}
        self.frame_count = 0
        self.last_image_size = (640, 480)

    def _image_cb(self, message: Image) -> None:
        self.frame_count += 1
        if self.model is None or self.frame_count % self.detection_stride != 0:
            self._publish_tracks(message.header.stamp)
            return

        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.last_image_size = (frame.shape[1], frame.shape[0])
        results = self.model.predict(frame, conf=self.conf_threshold, verbose=False)
        detections = []
        result = results[0]
        names = result.names
        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().tolist()
            confidences = result.boxes.conf.cpu().tolist()
            classes = result.boxes.cls.cpu().tolist()
            for bbox, confidence, cls_index in zip(boxes, confidences, classes, strict=False):
                label = names[int(cls_index)]
                estimated_distance_m = self._estimate_distance(frame.shape[0], bbox, label)
                detections.append(
                    {
                        "bbox": tuple(float(value) for value in bbox),
                        "confidence": float(confidence),
                        "label": str(label),
                        "estimated_distance_m": estimated_distance_m,
                    }
                )

        self._update_tracks(detections)
        self._publish_tracks(message.header.stamp)

    def _estimate_distance(self, image_height: int, bbox: list[float] | tuple[float, ...], label: str) -> float:
        pixel_height = max(1.0, float(bbox[3] - bbox[1]))
        expected_height = self.class_heights.get(label, 0.5)
        focal_length = image_height / (2.0 * math.tan(math.radians(self.camera_vertical_fov_deg) / 2.0))
        return float(expected_height * focal_length / pixel_height)

    def _update_tracks(self, detections: list[dict[str, Any]]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        unmatched = set(self.tracks.keys())
        for detection in detections:
            best_id = None
            best_iou = 0.0
            for track_id, track in self.tracks.items():
                if track.label != detection["label"]:
                    continue
                iou = self._iou(track.bbox, detection["bbox"])
                if iou > 0.3 and iou > best_iou:
                    best_id = track_id
                    best_iou = iou

            if best_id is None:
                best_id = uuid4().hex[:8]
            else:
                unmatched.discard(best_id)

            self.tracks[best_id] = TrackState(
                track_id=best_id,
                label=detection["label"],
                confidence=detection["confidence"],
                bbox=detection["bbox"],
                estimated_distance_m=detection["estimated_distance_m"],
                last_seen_ns=now_ns,
            )

        ttl_ns = int(self.track_ttl_sec * 1e9)
        for track_id in list(unmatched):
            if now_ns - self.tracks[track_id].last_seen_ns > ttl_ns:
                self.tracks.pop(track_id, None)

    def _publish_tracks(self, stamp) -> None:
        now_ns = self.get_clock().now().nanoseconds
        ttl_ns = int(self.track_ttl_sec * 1e9)
        message = TrackedObjectArray()
        message.header.stamp = stamp
        image_width, image_height = self.last_image_size
        for track_id in list(self.tracks):
            track = self.tracks[track_id]
            if now_ns - track.last_seen_ns > ttl_ns:
                self.tracks.pop(track_id, None)
                continue
            x1, y1, x2, y2 = track.bbox
            tracked = TrackedObject()
            tracked.header.stamp = stamp
            tracked.track_id = track.track_id
            tracked.label = track.label
            tracked.confidence = track.confidence
            tracked.center_x = float(((x1 + x2) / 2.0) / max(1.0, image_width))
            tracked.center_y = float(((y1 + y2) / 2.0) / max(1.0, image_height))
            tracked.width = float((x2 - x1) / max(1.0, image_width))
            tracked.height = float((y2 - y1) / max(1.0, image_height))
            tracked.estimated_distance_m = track.estimated_distance_m
            tracked.actively_tracked = True
            message.objects.append(tracked)
        self.publisher.publish(message)

    @staticmethod
    def _iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
            return 0.0
        inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
        a_area = (ax2 - ax1) * (ay2 - ay1)
        b_area = (bx2 - bx1) * (by2 - by1)
        return inter_area / max(1.0, a_area + b_area - inter_area)


def main() -> None:
    rclpy.init()
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
