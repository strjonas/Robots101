"""Camera geometry and box matching for the object tracker, kept free of ROS so it can be tested."""

from __future__ import annotations

import math


def bearing_from_image_x(pixel_x: float, image_width: int, horizontal_fov_rad: float) -> float:
    """Direction of an image column seen from the camera, radians, positive = left.

    Uses a pinhole camera: the image centre is straight ahead, and the edges are
    at +/- half the field of view.
    """
    focal_px = (image_width / 2.0) / math.tan(horizontal_fov_rad / 2.0)
    return math.atan((image_width / 2.0 - pixel_x) / focal_px)


def distance_from_box_height(box_height_px: float, image_height: int, vertical_fov_rad: float, real_height_m: float) -> float:
    """Guess the distance of an object of known real height from how tall it looks.

    Similar triangles: distance = real height * focal length / height in pixels.
    Overestimates when the box is cut off by the image edge.
    """
    focal_px = (image_height / 2.0) / math.tan(vertical_fov_rad / 2.0)
    return float(real_height_m * focal_px / max(1.0, box_height_px))


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    """Intersection over union of two boxes (x1, y1, x2, y2): 0 = disjoint, 1 = identical."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_w = min(ax2, bx2) - max(ax1, bx1)
    inter_h = min(ay2, by2) - max(ay1, by1)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0
    inter_area = inter_w * inter_h
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter_area
    return inter_area / max(1.0, union)
