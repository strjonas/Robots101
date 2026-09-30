import math

import pytest

from brain_nodes.tracking_logic import bearing_from_image_x, distance_from_box_height, iou

FOV = math.radians(60)


def test_bearing_is_zero_in_the_middle_and_half_fov_at_the_edges() -> None:
    assert bearing_from_image_x(320, 640, FOV) == pytest.approx(0.0)
    assert bearing_from_image_x(0, 640, FOV) == pytest.approx(FOV / 2)  # left edge -> positive
    assert bearing_from_image_x(640, 640, FOV) == pytest.approx(-FOV / 2)


def test_distance_from_height_halves_when_the_box_doubles() -> None:
    far = distance_from_box_height(100, 480, FOV, 1.7)
    near = distance_from_box_height(200, 480, FOV, 1.7)
    assert near == pytest.approx(far / 2)


def test_distance_matches_pinhole_geometry() -> None:
    # An object that exactly fills the image height is at real_height / (2 tan(fov/2)).
    assert distance_from_box_height(480, 480, FOV, 1.0) == pytest.approx(1.0 / (2 * math.tan(FOV / 2)))


def test_iou() -> None:
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    assert iou((0, 0, 10, 10), (5, 0, 15, 10)) == pytest.approx(50 / 150)
