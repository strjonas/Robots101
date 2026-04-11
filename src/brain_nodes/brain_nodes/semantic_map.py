from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Iterable, Optional

import yaml


@dataclass
class SemanticTarget:
    name: str
    x: float
    y: float
    yaw: float
    tags: list[str]


class SemanticMap:
    def __init__(self, frame_id: str, targets: list[SemanticTarget], patrol_order: list[str]):
        self.frame_id = frame_id
        self.targets = targets
        self.targets_by_name = {target.name: target for target in targets}
        self.patrol_order = patrol_order

    @classmethod
    def load(cls, path: Path | str) -> "SemanticMap":
        path = Path(path)
        if not path.exists():
            return cls(frame_id="map", targets=[], patrol_order=[])

        raw = yaml.safe_load(path.read_text()) or {}
        targets = [
            SemanticTarget(
                name=item["name"],
                x=float(item["x"]),
                y=float(item["y"]),
                yaw=float(item.get("yaw", 0.0)),
                tags=[str(tag) for tag in item.get("tags", [])],
            )
            for item in raw.get("targets", [])
        ]
        patrol_order = [str(name) for name in raw.get("patrol_order", [])]
        return cls(frame_id=str(raw.get("frame_id", "map")), targets=targets, patrol_order=patrol_order)

    def save(self, path: Path | str) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        document = {
            "frame_id": self.frame_id,
            "patrol_order": self.patrol_order,
            "targets": [
                {
                    "name": target.name,
                    "x": target.x,
                    "y": target.y,
                    "yaw": target.yaw,
                    "tags": target.tags,
                }
                for target in self.targets
            ],
        }
        path.write_text(yaml.safe_dump(document, sort_keys=False))

    def get(self, name: str) -> Optional[SemanticTarget]:
        return self.targets_by_name.get(name)

    def describe_targets(self, limit: int = 12) -> list[dict[str, Any]]:
        return [asdict(target) for target in self.targets[:limit]]

    def nearest_name(self, x: float, y: float, max_distance: float = 1.5) -> str:
        best_name = "unknown"
        best_distance = max_distance
        for target in self.targets:
            dx = target.x - x
            dy = target.y - y
            distance = (dx * dx + dy * dy) ** 0.5
            if distance < best_distance:
                best_name = target.name
                best_distance = distance
        return best_name

    def patrol_targets(self) -> Iterable[SemanticTarget]:
        if self.patrol_order:
            for name in self.patrol_order:
                target = self.targets_by_name.get(name)
                if target is not None:
                    yield target
            return

        for target in self.targets:
            if "patrol" in target.tags:
                yield target

    def upsert_target(self, name: str, x: float, y: float, yaw: float, tag: str) -> None:
        existing = self.targets_by_name.get(name)
        if existing is None:
            existing = SemanticTarget(name=name, x=x, y=y, yaw=yaw, tags=[tag] if tag else [])
            self.targets.append(existing)
            self.targets_by_name[name] = existing
        else:
            existing.x = x
            existing.y = y
            existing.yaw = yaw
            if tag and tag not in existing.tags:
                existing.tags.append(tag)
        if tag == "patrol" and name not in self.patrol_order:
            self.patrol_order.append(name)

