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
            "approach_steering": [0.0, -1.0, 0.0],  # Camera forward view direction
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
