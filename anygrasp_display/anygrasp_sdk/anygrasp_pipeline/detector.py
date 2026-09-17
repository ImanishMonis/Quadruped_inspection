import sys
import numpy as np

import config

# Make AnyGrasp SDK visible
sys.path.append(
    config.GRASP_DETECTION_SDK_PATH
)

from gsnet import create_detector


class GraspResult:
    """
    Clean grasp representation.
    """

    def __init__(self, grasp):

        self.score = float(grasp.score)
        self.width = float(grasp.width)
        self.height = float(grasp.height)
        self.depth = float(grasp.depth)

        self.translation = grasp.translation.copy()

        self.rotation = grasp.rotation_matrix.copy()


    def __str__(self):

        return (
            "\n"
            "====================\n"
            "Grasp Result\n"
            "====================\n"
            f"Score:\n{self.score}\n\n"
            f"Width:\n{self.width}\n\n"
            f"Depth:\n{self.depth}\n\n"
            f"Translation:\n{self.translation}\n\n"
            f"Rotation:\n{self.rotation}\n"
        )



class AnyGraspDetector:


    def __init__(
        self,
        checkpoint_path=None,
        max_gripper_width=config.MAX_GRIPPER_WIDTH,
        gripper_height=config.GRIPPER_HEIGHT,
    ):


        from argparse import Namespace


        cfg = Namespace()

        cfg.checkpoint_path = (
            checkpoint_path
            if checkpoint_path is not None
            else config.CHECKPOINT_PATH
        )
        cfg.max_gripper_width = max_gripper_width
        cfg.gripper_height = gripper_height


        self.detector = create_detector(cfg)


        if self.detector is None:
            raise RuntimeError(
                "Failed to initialize AnyGrasp"
            )


    def predict(
        self,
        points,
        collision_detection=True,
        region_mask=None,
    ):

        """
        Run AnyGrasp inference.

        points:
            numpy array (N,3)
            camera coordinate frame
            meters

        Returns
        -------
        GraspGroup, sorted best-first, or None if nothing was found.
        Per AGENT.md decision #5 the wrapper does NOT pick a single
        grasp - the caller chooses best/top5/top20/etc.
        """


        params = {

            "dense_grasp": False,
            "collision_detection":collision_detection,
            "region_steering":region_mask,
            # [0,-1,0] looked like a copy-paste from USAGE.md's generic
            # example (matches its "all controls combined" snippet
            # exactly), which briefly led to "fixing" this to [0,0,1] on
            # 2026-09-17, reasoning from the deprojection math alone
            # (textbook pinhole -> +Z forward). Reverted: the user had
            # already empirically validated [0,-1,0] - setting it to
            # [0,-1,0] visibly produces grasps approaching from the front
            # of the object, confirmed by watching real captures, which
            # is stronger evidence than the formula-based assumption.
            # This means the *actual* effective forward axis for this
            # camera's real published data is -Y, not the +Z the pinhole
            # formula alone would suggest - i.e. something about the live
            # depth image's actual row/column layout or how Isaac Sim
            # publishes it doesn't match the naive textbook assumption.
            # Not yet root-caused - see AGENT_SESSION.md. Don't "fix" this
            # back to [0,0,1] without re-validating empirically first.
            "approach_steering": [0.0, -1.0, 0.0],
            "approach_thresh": np.pi / 4,
        }


        grasps = self.detector.get_grasp(
            points.astype(np.float32),
            params
        )


        if grasps is None or len(grasps) == 0:
            return None


        grasps = grasps.nms()

        grasps = grasps.sort_by_score()

        # filter_parallel_grasps() disabled 2026-09-17: it crashed every
        # request (IndexError: tuple index out of range, from
        # `grasps[valid_indices]` - GraspGroup only reliably supports
        # single-int indexing and slicing, not a list of indices). It also
        # had the axis backwards (indexed row 0 when it meant column 0,
        # and even column 0 is this codebase's APPROACH axis, not the
        # "closing direction" the docstring claimed - see tf_utils.py) and
        # used a fixed camera-frame ground_normal, which only holds for
        # one specific camera pose since the camera moves with the arm.
        # "Parallel to the ground" now belongs entirely to the client
        # side instead: grasp_pipeline/grasp_planner.select_level_grasp()
        # correctly uses the approach axis, transforms into the WORLD
        # frame via live TF, and is tunable per-call
        # (--max-tilt-deg/--top-k in executor.py). See AGENT_SESSION.md.
        # grasps = filter_parallel_grasps(grasps)

        if len(grasps) == 0:
            return None


        return grasps


    def predict_all(
        self,
        points
    ):

        """
        Run AnyGrasp with no region mask and no collision context.
        Simple/exploratory mode - prefer predict() with the full scene
        and a region mask for anything collision-aware (AGENT.md #3/#4).
        """

        params = {
            "dense_grasp": False,
            "collision_detection": False,
            "region_steering": None,
        }

        grasps = self.detector.get_grasp(
            points.astype(np.float32),
            params,
        )


        if grasps is None or len(grasps) == 0:
            return None

        grasps = grasps.nms()

        return grasps.sort_by_score()


def filter_parallel_grasps(grasps, max_tilt_deg=10.0, ground_normal=np.array([0.0, 0.0, 1.0])):
    """
    Filters grasps so the gripper closing direction (R[:, 0]) is parallel to the ground.
    
    ground_normal: [0, -1, 0] in standard camera frame (+Y points down, so -Y points up).
    max_tilt_deg: Maximum allowed angle (in degrees) off the horizontal ground plane.
    """
    if grasps is None or len(grasps) == 0:
        return grasps

    # Normalize ground vector
    ground_normal = ground_normal / np.linalg.norm(ground_normal)
    max_allowed_vertical = np.sin(np.radians(max_tilt_deg))

    valid_indices = []
    for i in range(len(grasps)):
        # Column 0 is the finger closing direction vector
        closing_axis = grasps[i].rotation_matrix[0, :]
        
        # Absolute dot product measures vertical component (0 = perfectly horizontal)
        vertical_comp = abs(np.dot(closing_axis, ground_normal))
        
        if vertical_comp <= max_allowed_vertical:
            valid_indices.append(i)

    if len(valid_indices) == 0:
        print("[Warning] No grasps passed the parallel filter! Returning top unfiltered grasp.")
        return grasps

    return grasps[valid_indices]