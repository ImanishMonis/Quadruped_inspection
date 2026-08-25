#!/usr/bin/env python3
"""
HTTP client for the AnyGrasp container.

The MoveIt/ROS container and the AnyGrasp container are two deliberately
independent Docker containers talking over plain HTTP (AGENT.md). This
module knows nothing about ROS beyond using rospy for logging/params -
it just POSTs scene.pcd + object.pcd and returns the parsed grasp list,
sorted best-first, exactly as AnyGrasp scored them (AGENT.md decision #5:
the caller picks best/top5/top20, not this client).
"""

import rospy
import requests


DEFAULT_SERVER_URL = "http://172.17.0.2:5000"
# NOTE: the AnyGrasp container currently publishes no host port and sits
# on the default docker bridge network, so this IP is only stable until
# the container is recreated (docker inspect anygrasp_display to check).
# Override via the ~server_url param or ANYGRASP_SERVER_URL env var
# rather than editing this default.

DEFAULT_TIMEOUT_SEC = 30.0


class GraspClientError(RuntimeError):
    pass


def get_grasps(
    scene_path,
    object_path,
    server_url=DEFAULT_SERVER_URL,
    top_k=5,
    timeout=DEFAULT_TIMEOUT_SEC,
):
    """
    POST scene + object point clouds to the AnyGrasp /predict endpoint.

    Parameters
    ----------
    scene_path : str
        Path to scene.pcd (camera frame, meters).
    object_path : str
        Path to object.pcd (same camera frame as scene.pcd).
    server_url : str
        Base URL of the AnyGrasp container, e.g. "http://172.17.0.2:5000".
    top_k : int
        How many grasps to ask the server for, best-first.
    timeout : float
        HTTP timeout in seconds. Inference can be slow on first call
        (model/checkpoint already loaded at server startup, so this is
        just the actual grasp inference time).

    Returns
    -------
    list of dict
        Each dict has: score, width, depth, translation (list[3]),
        rotation (3x3 list of lists). Sorted best score first.
        Empty list if no grasp was found.

    Raises
    ------
    GraspClientError
        On a network/HTTP failure, a malformed response, or if the
        server itself reported an error.
    """

    url = server_url.rstrip("/") + "/predict"

    try:
        with open(scene_path, "rb") as scene_file, \
                open(object_path, "rb") as object_file:

            response = requests.post(
                url,
                files={
                    "scene": scene_file,
                    "object": object_file,
                },
                data={
                    "top_k": str(top_k),
                },
                timeout=timeout,
            )

    except requests.exceptions.RequestException as exc:
        raise GraspClientError(
            f"Failed to reach AnyGrasp server at {url}: {exc}"
        ) from exc

    if response.status_code != 200:
        raise GraspClientError(
            f"AnyGrasp server returned HTTP {response.status_code}: "
            f"{response.text}"
        )

    try:
        payload = response.json()
    except ValueError as exc:
        raise GraspClientError(
            f"AnyGrasp server returned non-JSON response: {response.text}"
        ) from exc

    if not payload.get("success", False):
        rospy.logwarn(
            "AnyGrasp reported no grasp found: %s",
            payload.get("message", "<no message>"),
        )
        return []

    grasps = payload.get("grasps", [])

    rospy.loginfo(
        "AnyGrasp returned %d grasp(s), best score %.4f",
        len(grasps),
        grasps[0]["score"] if grasps else float("nan"),
    )

    return grasps


def get_best_grasp(scene_path, object_path, server_url=DEFAULT_SERVER_URL,
                    timeout=DEFAULT_TIMEOUT_SEC):
    """
    Convenience wrapper: returns the single best grasp dict, or None.
    """

    grasps = get_grasps(
        scene_path,
        object_path,
        server_url=server_url,
        top_k=1,
        timeout=timeout,
    )

    return grasps[0] if grasps else None


def health_check(server_url=DEFAULT_SERVER_URL, timeout=5.0):
    """
    Returns True if the AnyGrasp server responds to GET /.
    """

    try:
        response = requests.get(
            server_url.rstrip("/") + "/",
            timeout=timeout,
        )
        return (
            response.status_code == 200
            and response.json().get("status") == "running"
        )
    except requests.exceptions.RequestException:
        return False


if __name__ == "__main__":

    import argparse
    import json

    parser = argparse.ArgumentParser(
        description="Standalone test: call the AnyGrasp REST API directly."
    )
    parser.add_argument("--scene", required=True)
    parser.add_argument("--object", required=True)
    parser.add_argument(
        "--server-url",
        default=rospy.get_param("~server_url", DEFAULT_SERVER_URL),
    )
    parser.add_argument("--top-k", type=int, default=5)
    args = parser.parse_args()

    rospy.init_node("grasp_client_test", anonymous=True)

    if not health_check(args.server_url):
        rospy.logerr(
            "AnyGrasp server at %s is not responding to health check",
            args.server_url,
        )
        raise SystemExit(1)

    grasps = get_grasps(
        args.scene,
        args.object,
        server_url=args.server_url,
        top_k=args.top_k,
    )
    
    print(json.dumps(grasps, indent=2))
