import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


WORKSPACE_ROOT = Path(os.environ.get("ROBOTS101_ROOT", Path(__file__).resolve().parents[3])).resolve()

TOPIC_CMD_VEL = "/brain/cmd_vel_final"
TOPIC_CMD_VEL_STAMPED = "/cmd_vel"
TOPIC_CMD_VEL_MANUAL = "/brain/cmd_vel_manual"
TOPIC_CMD_VEL_NAV = "/brain/cmd_vel_nav"
TOPIC_CMD_VEL_EXECUTOR = "/brain/cmd_vel_executor"

TOPIC_CONTROL_MODE = "/brain/control_mode"
TOPIC_CURRENT_TASK = "/brain/current_task"
TOPIC_EXECUTOR_STATUS = "/brain/executor_status"
TOPIC_PLANNER_ACTION = "/brain/planner_action"
TOPIC_OBSERVATION_SUMMARY = "/brain/observation_summary"
TOPIC_TRACKED_OBJECTS = "/brain/tracked_objects"
TOPIC_SAFETY_STOP = "/brain/safety_stop"
TOPIC_ACTIVE_CMD_SOURCE = "/brain/active_cmd_source"
TOPIC_SEMANTIC_MARKERS = "/brain/semantic_markers"
TOPIC_DETECTION_IMAGE = "/brain/detection_image"

SERVICE_SUBMIT_TASK = "/brain/submit_task"
SERVICE_CLEAR_TASK = "/brain/clear_task"
SERVICE_SET_EMERGENCY_STOP = "/brain/set_emergency_stop"
SERVICE_RECORD_SEMANTIC_TARGET = "/brain/record_semantic_target"

def _package_path(relative_path: str) -> Path:
    # Prefer the source tree so edits to config files apply without rebuilding.
    source_path = WORKSPACE_ROOT / "src" / "brain_bringup" / relative_path
    if source_path.is_file():
        return source_path
    try:
        return Path(get_package_share_directory("brain_bringup")) / relative_path
    except PackageNotFoundError:
        return source_path


DEFAULT_SEMANTIC_MAP_PATH = _package_path("config/semantic_map.yaml")
DEFAULT_PROMPT_PATH = _package_path("config/gemma_system_prompt.md")

ACTION_NAME_TO_TYPE = {
    "STOP": 0,
    "TURN": 1,
    "DRIVE": 2,
    "GOTO_SEMANTIC": 3,
    "FOLLOW_OBJECT": 4,
    "LOOK_AROUND": 5,
}

ACTION_TYPE_TO_NAME = {value: key for key, value in ACTION_NAME_TO_TYPE.items()}
