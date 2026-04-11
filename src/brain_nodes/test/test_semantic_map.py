from pathlib import Path

from brain_nodes.semantic_map import SemanticMap


def test_load_and_upsert_semantic_map(tmp_path: Path) -> None:
    path = tmp_path / "semantic_map.yaml"
    path.write_text(
        """
frame_id: map
patrol_order: [a]
targets:
  - name: a
    x: 1.0
    y: 2.0
    yaw: 0.0
    tags: [patrol]
"""
    )
    semantic_map = SemanticMap.load(path)
    assert semantic_map.frame_id == "map"
    assert semantic_map.nearest_name(1.1, 2.0) == "a"

    semantic_map.upsert_target("b", 3.0, 4.0, 0.5, "patrol")
    semantic_map.save(path)
    saved = SemanticMap.load(path)
    assert saved.get("b") is not None
