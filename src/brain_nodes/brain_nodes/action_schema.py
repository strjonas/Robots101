from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field, model_validator


class ActionProposal(BaseModel):
    action: str = Field(description="One of STOP, TURN, DRIVE, GOTO_SEMANTIC, FOLLOW_OBJECT, LOOK_AROUND")
    angle_rad: float = 0.0
    distance_m: float = 0.0
    speed_limit: float = 0.2
    target_id: str = ""
    follow_label: str = ""
    follow_track_id: str = ""
    preferred_distance_m: float = 0.8
    confidence: float = 0.0
    rationale: str = ""

    @model_validator(mode="after")
    def validate_shape(self) -> "ActionProposal":
        allowed = {"STOP", "TURN", "DRIVE", "GOTO_SEMANTIC", "FOLLOW_OBJECT", "LOOK_AROUND"}
        if self.action not in allowed:
            raise ValueError(f"Unsupported action '{self.action}'")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")
        if self.action == "TURN" and abs(self.angle_rad) < 1e-3:
            raise ValueError("TURN requires a non-zero angle_rad")
        if self.action == "DRIVE" and abs(self.distance_m) < 1e-3:
            raise ValueError("DRIVE requires a non-zero distance_m")
        if self.action == "GOTO_SEMANTIC" and not self.target_id:
            raise ValueError("GOTO_SEMANTIC requires target_id")
        if self.action == "FOLLOW_OBJECT" and not (self.follow_track_id or self.follow_label):
            raise ValueError("FOLLOW_OBJECT requires follow_track_id or follow_label")
        return self


class ParsedModelResult(BaseModel):
    proposal: Optional[ActionProposal] = None
    raw_content: str = ""
    error: str = ""

