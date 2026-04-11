from __future__ import annotations

import base64
import json
import re
from pathlib import Path

import requests

from brain_nodes.action_schema import ActionProposal, ParsedModelResult
from brain_nodes.model_clients.base import ModelRequest


class OllamaClient:
    def __init__(self, model_name: str, ollama_url: str, system_prompt_path: str) -> None:
        self.model_name = model_name
        self.ollama_url = ollama_url.rstrip("/")
        self.system_prompt = Path(system_prompt_path).read_text().strip()

    def infer(self, request: ModelRequest) -> ParsedModelResult:
        user_payload = {
            "task_id": request.task_id,
            "instruction": request.instruction,
            "target_hint": request.target_hint,
            "mode": request.mode_name,
            "observation": request.observation,
            "semantic_targets": request.semantic_targets,
            "required_output_schema": {
                "action": "STOP | TURN | DRIVE | GOTO_SEMANTIC | FOLLOW_OBJECT | LOOK_AROUND",
                "angle_rad": "float",
                "distance_m": "float",
                "speed_limit": "float",
                "target_id": "string",
                "follow_label": "string",
                "follow_track_id": "string",
                "preferred_distance_m": "float",
                "confidence": "0.0-1.0",
                "rationale": "string",
            },
        }
        message = {"role": "user", "content": json.dumps(user_payload, indent=2)}
        if request.image_bytes:
            message["images"] = [base64.b64encode(request.image_bytes).decode("utf-8")]

        payload = {
            "model": self.model_name,
            "stream": False,
            "format": "json",
            "messages": [
                {"role": "system", "content": self.system_prompt},
                message,
            ],
        }

        response = requests.post(f"{self.ollama_url}/api/chat", json=payload, timeout=120)
        response.raise_for_status()
        content = response.json()["message"]["content"]
        cleaned = self._extract_json(content)
        try:
            proposal = ActionProposal.model_validate_json(cleaned)
            return ParsedModelResult(proposal=proposal, raw_content=content)
        except Exception as exc:  # noqa: BLE001
            return ParsedModelResult(raw_content=content, error=str(exc))

    @staticmethod
    def _extract_json(text: str) -> str:
        stripped = text.strip()
        if stripped.startswith("{") and stripped.endswith("}"):
            return stripped
        fenced = re.search(r"```json\s*(\{.*\})\s*```", stripped, re.DOTALL)
        if fenced:
            return fenced.group(1)
        first = stripped.find("{")
        last = stripped.rfind("}")
        if first >= 0 and last > first:
            return stripped[first:last + 1]
        return stripped

