#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class BoxObstacle:
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float


def parse_pose(text: str | None) -> Pose2D:
    if not text:
        return Pose2D()
    values = [float(token) for token in text.split()]
    while len(values) < 6:
        values.append(0.0)
    return Pose2D(x=values[0], y=values[1], z=values[2], yaw=values[5])


def compose_pose(base: Pose2D, child: Pose2D) -> Pose2D:
    cos_yaw = math.cos(base.yaw)
    sin_yaw = math.sin(base.yaw)
    x = base.x + cos_yaw * child.x - sin_yaw * child.y
    y = base.y + sin_yaw * child.x + cos_yaw * child.y
    return Pose2D(x=x, y=y, z=base.z + child.z, yaw=base.yaw + child.yaw)


def intersects_robot_slice(center_z: float, size_z: float, max_obstacle_height: float, min_slice_height: float) -> bool:
    bottom = center_z - size_z / 2.0
    top = center_z + size_z / 2.0
    return bottom < max_obstacle_height and top > min_slice_height


def collect_obstacles(
    model_element: ET.Element,
    base_pose: Pose2D,
    max_obstacle_height: float,
    min_slice_height: float,
) -> tuple[list[BoxObstacle | CircleObstacle], int]:
    obstacles: list[BoxObstacle | CircleObstacle] = []
    ignored_meshes = 0
    model_pose = compose_pose(base_pose, parse_pose(model_element.findtext("pose")))

    for link in model_element.findall("link"):
        link_pose = compose_pose(model_pose, parse_pose(link.findtext("pose")))
        for collision in link.findall("collision"):
            collision_pose = compose_pose(link_pose, parse_pose(collision.findtext("pose")))
            geometry = collision.find("geometry")
            if geometry is None:
                continue

            box = geometry.find("box")
            if box is not None:
                size = [float(token) for token in box.findtext("size", default="0 0 0").split()]
                if len(size) != 3 or not intersects_robot_slice(
                    collision_pose.z,
                    size[2],
                    max_obstacle_height,
                    min_slice_height,
                ):
                    continue
                obstacles.append(
                    BoxObstacle(
                        x=collision_pose.x,
                        y=collision_pose.y,
                        yaw=collision_pose.yaw,
                        size_x=size[0],
                        size_y=size[1],
                    )
                )
                continue

            cylinder = geometry.find("cylinder")
            if cylinder is not None:
                radius = float(cylinder.findtext("radius", default="0"))
                length = float(cylinder.findtext("length", default="0"))
                if radius <= 0.0 or not intersects_robot_slice(
                    collision_pose.z,
                    length,
                    max_obstacle_height,
                    min_slice_height,
                ):
                    continue
                obstacles.append(CircleObstacle(x=collision_pose.x, y=collision_pose.y, radius=radius))
                continue

            if geometry.find("mesh") is not None:
                ignored_meshes += 1

    for child_model in model_element.findall("model"):
        child_obstacles, child_ignored_meshes = collect_obstacles(
            child_model,
            model_pose,
            max_obstacle_height,
            min_slice_height,
        )
        obstacles.extend(child_obstacles)
        ignored_meshes += child_ignored_meshes

    return obstacles, ignored_meshes


def box_bounds(obstacle: BoxObstacle) -> tuple[float, float, float, float]:
    half_x = obstacle.size_x / 2.0
    half_y = obstacle.size_y / 2.0
    cos_yaw = math.cos(obstacle.yaw)
    sin_yaw = math.sin(obstacle.yaw)
    corners = []
    for local_x, local_y in ((half_x, half_y), (half_x, -half_y), (-half_x, half_y), (-half_x, -half_y)):
        world_x = obstacle.x + cos_yaw * local_x - sin_yaw * local_y
        world_y = obstacle.y + sin_yaw * local_x + cos_yaw * local_y
        corners.append((world_x, world_y))
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    return min(xs), min(ys), max(xs), max(ys)


def circle_bounds(obstacle: CircleObstacle) -> tuple[float, float, float, float]:
    return (
        obstacle.x - obstacle.radius,
        obstacle.y - obstacle.radius,
        obstacle.x + obstacle.radius,
        obstacle.y + obstacle.radius,
    )


def obstacle_bounds(obstacles: list[BoxObstacle | CircleObstacle], padding_m: float) -> tuple[float, float, float, float]:
    min_x = math.inf
    min_y = math.inf
    max_x = -math.inf
    max_y = -math.inf

    for obstacle in obstacles:
        bounds = box_bounds(obstacle) if isinstance(obstacle, BoxObstacle) else circle_bounds(obstacle)
        min_x = min(min_x, bounds[0])
        min_y = min(min_y, bounds[1])
        max_x = max(max_x, bounds[2])
        max_y = max(max_y, bounds[3])

    if not obstacles:
        min_x = min_y = -2.0
        max_x = max_y = 2.0

    return (
        min_x - padding_m,
        min_y - padding_m,
        max_x + padding_m,
        max_y + padding_m,
    )


def rasterize_obstacles(
    obstacles: list[BoxObstacle | CircleObstacle],
    resolution: float,
    bounds: tuple[float, float, float, float],
) -> tuple[np.ndarray, float, float]:
    min_x, min_y, max_x, max_y = bounds
    width = max(1, math.ceil((max_x - min_x) / resolution))
    height = max(1, math.ceil((max_y - min_y) / resolution))
    grid = np.full((height, width), 254, dtype=np.uint8)

    for obstacle in obstacles:
        bbox = box_bounds(obstacle) if isinstance(obstacle, BoxObstacle) else circle_bounds(obstacle)
        min_col = max(0, int(math.floor((bbox[0] - min_x) / resolution)))
        max_col = min(width - 1, int(math.ceil((bbox[2] - min_x) / resolution)))
        min_row = max(0, int(math.floor((max_y - bbox[3]) / resolution)))
        max_row = min(height - 1, int(math.ceil((max_y - bbox[1]) / resolution)))
        if min_col > max_col or min_row > max_row:
            continue

        cols = np.arange(min_col, max_col + 1)
        rows = np.arange(min_row, max_row + 1)
        world_x = min_x + (cols + 0.5) * resolution
        world_y = max_y - (rows + 0.5) * resolution
        xx, yy = np.meshgrid(world_x, world_y)

        if isinstance(obstacle, BoxObstacle):
            dx = xx - obstacle.x
            dy = yy - obstacle.y
            cos_yaw = math.cos(obstacle.yaw)
            sin_yaw = math.sin(obstacle.yaw)
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            mask = (np.abs(local_x) <= obstacle.size_x / 2.0) & (np.abs(local_y) <= obstacle.size_y / 2.0)
        else:
            mask = (xx - obstacle.x) ** 2 + (yy - obstacle.y) ** 2 <= obstacle.radius**2

        subgrid = grid[min_row : max_row + 1, min_col : max_col + 1]
        subgrid[mask] = 0

    return grid, min_x, min_y


def write_pgm(path: Path, grid: np.ndarray) -> None:
    with path.open("wb") as handle:
        handle.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode("ascii"))
        handle.write(grid.tobytes())


def default_model_path(repo_root: Path) -> Path:
    if "CONDA_PREFIX" in os.environ:
        return Path(os.environ["CONDA_PREFIX"]) / "share" / "turtlebot3_gazebo" / "models" / "turtlebot3_house" / "model.sdf"
    return repo_root / ".pixi" / "envs" / "default" / "share" / "turtlebot3_gazebo" / "models" / "turtlebot3_house" / "model.sdf"


def main() -> None:
    repo_root = Path(os.environ.get("ROBOTS101_ROOT", Path(__file__).resolve().parents[1])).resolve()
    parser = argparse.ArgumentParser(description="Generate a Nav2 occupancy map from the TurtleBot3 house SDF model.")
    parser.add_argument("--model", type=Path, default=default_model_path(repo_root))
    parser.add_argument("--output-yaml", type=Path, default=repo_root / "maps" / "turtlebot3_house.yaml")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding-m", type=float, default=0.75)
    parser.add_argument("--max-obstacle-height", type=float, default=0.35)
    parser.add_argument("--min-slice-height", type=float, default=0.02)
    args = parser.parse_args()

    model_path = args.model.resolve()
    output_yaml = args.output_yaml.resolve()
    output_pgm = output_yaml.with_suffix(".pgm")
    output_yaml.parent.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(model_path)
    root = tree.getroot()
    model_element = root.find(".//model")
    if model_element is None:
        raise RuntimeError(f"No <model> element found in {model_path}")

    obstacles, ignored_meshes = collect_obstacles(
        model_element,
        Pose2D(),
        max_obstacle_height=args.max_obstacle_height,
        min_slice_height=args.min_slice_height,
    )
    bounds = obstacle_bounds(obstacles, padding_m=args.padding_m)
    grid, origin_x, origin_y = rasterize_obstacles(obstacles, resolution=args.resolution, bounds=bounds)

    write_pgm(output_pgm, grid)
    yaml_payload = {
        "image": output_pgm.name,
        "resolution": args.resolution,
        "origin": [round(origin_x, 6), round(origin_y, 6), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }
    output_yaml.write_text(yaml.safe_dump(yaml_payload, sort_keys=False), encoding="utf-8")

    occupied_ratio = float(np.count_nonzero(grid == 0)) / float(grid.size)
    print(f"Generated {output_yaml}")
    print(f"Model: {model_path}")
    print(f"Grid: {grid.shape[1]} x {grid.shape[0]} @ {args.resolution:.3f} m")
    print(f"Origin: ({origin_x:.3f}, {origin_y:.3f})")
    print(f"Obstacles: {len(obstacles)}")
    print(f"Ignored mesh collisions: {ignored_meshes}")
    print(f"Occupied ratio: {occupied_ratio:.3f}")


if __name__ == "__main__":
    main()
