import os

# Path to the AnyGrasp SDK's grasp_detection module (contains gsnet.py).
# Hardcoded to the container's own layout by default; override for other setups.
GRASP_DETECTION_SDK_PATH = os.environ.get(
    "ANYGRASP_SDK_PATH",
    "/workspace/anygrasp_sdk/grasp_detection",
)

# Path to the licensed detection checkpoint.
CHECKPOINT_PATH = os.environ.get(
    "ANYGRASP_CHECKPOINT_PATH",
    "/workspace/checkpoint_detection.tar",
)

MAX_GRIPPER_WIDTH = float(
    os.environ.get("ANYGRASP_MAX_GRIPPER_WIDTH", "0.7")
)

GRIPPER_HEIGHT = float(
    os.environ.get("ANYGRASP_GRIPPER_HEIGHT", "0.03")
)

# Max distance (meters) for a scene point to count as part of the
# selected object when building the region mask. See mask.py.
DEFAULT_MASK_THRESHOLD = float(
    os.environ.get("ANYGRASP_MASK_THRESHOLD", "0.005")
)

# How many grasps /predict returns by default when the caller doesn't
# specify top_k. AGENT.md decision #5: the wrapper returns a GraspGroup,
# the caller picks best/top5/top20 - this is the server's own default pick.
DEFAULT_TOP_K = int(
    os.environ.get("ANYGRASP_TOP_K", "5")
)
