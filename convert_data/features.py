def build_features(camera_shapes: dict, state_dim: int) -> dict:
    """LeRobot feature schema.

    Args:
        camera_shapes: camera name -> (height, width, channel) image shape, in robot camera order.
        state_dim: state/action size, 7 per arm (14 for the dual robot, 7 for the single robot).
    """
    features = {
        f"observation.images.{name}": {
            "dtype": "video",
            "shape": list(shape),
            "names": ["height", "width", "channel"],
        }
        for name, shape in camera_shapes.items()
    }
    features.update(
        {
            "observation.state": {
                "dtype": "float32",
                "shape": (state_dim,),
            },
            "observation.eef_pose": {
                "dtype": "float32",
                "shape": (state_dim,),
            },
            "eef_action": {
                "dtype": "float32",
                "shape": (state_dim,),
            },
            "action": {
                "dtype": "float32",
                "shape": (state_dim,),
            },
            "collect": {
                "dtype": "string",
                "shape": (1,),
            },
        }
    )
    return features
