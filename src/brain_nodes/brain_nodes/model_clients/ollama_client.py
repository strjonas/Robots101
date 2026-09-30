"""Planner backend that asks a local LLM served by Ollama.

The model receives the system prompt (config/gemma_system_prompt.md), one JSON
document describing the task and the robot's situation, and the current camera
image. It must answer with one JSON action, which is validated by ActionProposal
before anything reaches the robot.
"""

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

    def check(self) -> str:
        """Quick health check. Returns "" if the server is up and has the model, else what is wrong."""
        try:
            version = requests.get(f"{self.ollama_url}/api/version", timeout=3).json().get("version", "?")
            models = requests.get(f"{self.ollama_url}/api/tags", timeout=3).json().get("models", [])
        except requests.RequestException:
            return f"Ollama is not reachable at {self.ollama_url}. Start the Ollama app or `ollama serve`."
        names = {model.get("name") for model in models}
        if self.model_name not in names and f"{self.model_name}:latest" not in names:
            return f"Ollama {version} does not have '{self.model_name}'. Run `ollama pull {self.model_name}`."
        return ""

    def infer(self, request: ModelRequest) -> ParsedModelResult:
        user_payload = {
            "task_id": request.task_id,
            "instruction": request.instruction,
            "target_hint": request.target_hint,
            "mode": request.mode_name,
            "observation": request.observation,
            "semantic_targets": request.semantic_targets,
            "history": request.history,
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
            "options": {"temperature": 0},
            "messages": [
                {"role": "system", "content": self.system_prompt},
                message,
            ],
        }

        try:
            response = requests.post(f"{self.ollama_url}/api/chat", json=payload, timeout=120)
        except requests.RequestException as exc:
            return ParsedModelResult(error=f"cannot reach Ollama at {self.ollama_url}: {exc}")
        if not response.ok:
            # Ollama explains what went wrong in the body, e.g. a missing or unsupported model.
            return ParsedModelResult(error=f"Ollama HTTP {response.status_code}: {response.text.strip()[:300]}")
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

