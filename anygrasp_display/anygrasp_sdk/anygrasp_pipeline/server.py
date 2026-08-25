from flask import Flask, request, jsonify
import tempfile
import os
import gc
import torch
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"

import config
from loader import load_scene, load_object
from mask import compute_object_mask
from detector import AnyGraspDetector, GraspResult

app = Flask(__name__)

print("Loading AnyGrasp model...")

detector = AnyGraspDetector(
    checkpoint_path=config.CHECKPOINT_PATH
)

print("Model loaded.")

@app.get("/")
def health():
    return jsonify(
        {
            "status": "running",
            "service": "AnyGrasp"
        }
    )


@app.post("/predict")
def predict():

    if "scene" not in request.files:
        return jsonify({"error": "Missing scene file"}), 400

    if "object" not in request.files:
        return jsonify({"error": "Missing object file"}), 400

    scene_file = request.files["scene"]
    object_file = request.files["object"]

    top_k = int(
        request.form.get("top_k", config.DEFAULT_TOP_K)
    )

    with tempfile.TemporaryDirectory() as tmp:

        scene_path = os.path.join(tmp, "scene.pcd")
        object_path = os.path.join(tmp, "object.pcd")

        scene_file.save(scene_path)
        object_file.save(object_path)

        scene = load_scene(scene_path)
        obj = load_object(object_path)

        mask = compute_object_mask(
            scene.points,
            obj.points,
            threshold=config.DEFAULT_MASK_THRESHOLD,
        )

        # Clear residual memory from previous runs before launching inference
        gc.collect()
        torch.cuda.empty_cache()
        # Collision detection is performed against the ENTIRE scene.
        # The object cloud is only used to build the region mask above
        # (AGENT.md decisions #3/#4) - never passed into AnyGrasp itself.
        grasps = detector.predict(
            scene.points,
            collision_detection=True,
            region_mask=mask,
        )

        if grasps is None or len(grasps) == 0:

            return jsonify(
                {
                    "success": False,
                    "message": "No grasp found"
                }
            )

        top = [
            GraspResult(g)
            for g in grasps[:top_k]
        ]

        return jsonify(
            {
                "success": True,
                "grasps": [
                    {
                        "score": g.score,
                        "width": g.width,
                        "depth": g.depth,
                        "translation": g.translation.tolist(),
                        "rotation": g.rotation.tolist(),
                    }
                    for g in top
                ],
            }
        )


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000
    )
