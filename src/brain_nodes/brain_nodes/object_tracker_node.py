"""Object tracker: finds objects in the camera image, keeps an id on each, and measures how far away they are.

Subscribes:  /camera/image_raw, /scan
Publishes:   /brain/tracked_objects  (label, box, bearing, distance)
             /brain/detection_image  (camera image with the boxes drawn, for RViz)

Detection is YOLO (a pretrained neural network, 80 everyday object classes).
Tracking is deliberately simple: a new box that overlaps an old box of the same
label keeps that box's id.

Distance is a small example of sensor fusion. The camera knows *which direction*
an object is in (from where its box sits in the image) but not how far away. The
lidar knows distances in every direction but not what is there. So we turn the
box position into a bearing and read the lidar at that bearing. If the lidar has
nothing there (out of range, or the object is too low for the lidar plane), we
fall back to guessing from the box height and a typical size for that class.
"""

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
from sensor_msgs.msg import Image, LaserScan

from brain_nodes.constants import TOPIC_DETECTION_IMAGE, TOPIC_TRACKED_OBJECTS
from brain_nodes.scan_utils import sector_min
from brain_nodes.tracking_logic import bearing_from_image_x, distance_from_box_height, iou

try:
    from ultralytics import YOLO
except Exception:  # noqa: BLE001
    YOLO = None

# Typical real-world heights in metres, for the size-based distance guess.
CLASS_HEIGHTS_M = {
    "person": 1.7,
    "chair": 0.9,
    "bottle": 0.25,
    "cup": 0.1,
    "book": 0.24,
    "tv": 0.6,
    "potted plant": 0.7,
    "bed": 0.6,
}


@dataclass
class TrackState:
    track_id: str
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    bearing_rad: float
    distance_m: float
    distance_source: str
    last_seen_ns: int


class ObjectTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("object_tracker")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("detection_stride", 3)
        self.declare_parameter("track_ttl_sec", 1.0)
        # Field of view of the TurtleBot3 Waffle Pi camera (from its model.sdf).
        self.declare_parameter("camera_horizontal_fov_rad", 1.085595)
        self.declare_parameter("camera_vertical_fov_rad", 0.8506)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.detection_stride = int(self.get_parameter("detection_stride").value)
        self.track_ttl_sec = float(self.get_parameter("track_ttl_sec").value)
        self.horizontal_fov = float(self.get_parameter("camera_horizontal_fov_rad").value)
        self.vertical_fov = float(self.get_parameter("camera_vertical_fov_rad").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(TrackedObjectArray, TOPIC_TRACKED_OBJECTS, 10)
        self.image_publisher = self.create_publisher(Image, TOPIC_DETECTION_IMAGE, 2)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)

        self.model = None
        if YOLO is not None:
            self.model = YOLO(str(self.get_parameter("model_name").value))
        else:
            self.get_logger().warning("Ultralytics is unavailable; tracker will publish empty detections.")

        self.scan: LaserScan | None = None
        self.tracks: dict[str, TrackState] = {}
        self.frame_count = 0
        self.last_image_size = (640, 480)

    def _scan_cb(self, message: LaserScan) -> None:
        self.scan = message

    def _image_cb(self, message: Image) -> None:
        self.frame_count += 1
        if self.model is None or self.frame_count % self.detection_stride != 0:
            self._publish_tracks(message.header.stamp)
            return

        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        image_height, image_width = frame.shape[:2]
        self.last_image_size = (image_width, image_height)
        result = self.model.predict(frame, conf=self.conf_threshold, verbose=False)[0]

        detections = []
        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().tolist()
            confidences = result.boxes.conf.cpu().tolist()
            classes = result.boxes.cls.cpu().tolist()
            for bbox, confidence, class_index in zip(boxes, confidences, classes, strict=False):
                label = str(result.names[int(class_index)])
                bearing, distance, source = self._locate(bbox, label, image_width, image_height)
                detections.append(
                    {
                        "bbox": tuple(float(value) for value in bbox),
                        "confidence": float(confidence),
                        "label": label,
                        "bearing_rad": bearing,
                        "distance_m": distance,
                        "distance_source": source,
                    }
                )

        self._update_tracks(detections)
        self._publish_tracks(message.header.stamp)
        self._publish_detection_image(frame, message.header)

    def _locate(self, bbox, label: str, image_width: int, image_height: int) -> tuple[float, float, str]:
        """Bearing and distance of one detection, preferring a lidar measurement."""
        x1, y1, x2, y2 = bbox
        bearing = bearing_from_image_x((x1 + x2) / 2.0, image_width, self.horizontal_fov)
        if self.scan is not None:
            # Look in a narrow cone around the box centre: a quarter of the box's angular width, at least 2 degrees.
            half_width = max(math.radians(2.0), 0.25 * (x2 - x1) / image_width * self.horizontal_fov)
            measured = sector_min(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment, bearing, half_width)
            if math.isfinite(measured) and measured < self.scan.range_max:
                return bearing, measured, "lidar"
        guessed = distance_from_box_height(y2 - y1, image_height, self.vertical_fov, CLASS_HEIGHTS_M.get(label, 0.5))
        return bearing, guessed, "size"

    def _update_tracks(self, detections: list[dict[str, Any]]) -> None:
        now_ns = self.get_clock().now().nanoseconds
        unmatched = set(self.tracks.keys())
        for detection in detections:
            best_id = None
            best_iou = 0.0
            for track_id, track in self.tracks.items():
                if track.label != detection["label"]:
                    continue
                overlap = iou(track.bbox, detection["bbox"])
                if overlap > 0.3 and overlap > best_iou:
                    best_id = track_id
                    best_iou = overlap

            if best_id is None:
                best_id = uuid4().hex[:8]
            else:
                unmatched.discard(best_id)

            self.tracks[best_id] = TrackState(track_id=best_id, last_seen_ns=now_ns, **detection)

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
            tracked.bearing_rad = track.bearing_rad
            tracked.estimated_distance_m = track.distance_m
            tracked.distance_source = track.distance_source
            tracked.actively_tracked = True
            message.objects.append(tracked)
        self.publisher.publish(message)

    def _publish_detection_image(self, frame, header) -> None:
        if self.image_publisher.get_subscription_count() == 0:
            return  # nobody is watching, skip the drawing
        for track in self.tracks.values():
            x1, y1, x2, y2 = (int(value) for value in track.bbox)
            caption = f"{track.label} {track.confidence:.2f} {track.distance_m:.1f}m ({track.distance_source})"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, caption, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated.header = header
        self.image_publisher.publish(annotated)


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
