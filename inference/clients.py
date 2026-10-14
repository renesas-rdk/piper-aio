import numpy as np
from openpi_client import image_tools, websocket_client_policy


def _random_observation(camera_names, state_dim) -> dict:
    return {
        "state": np.ones((state_dim,)),
        "images": {name: np.random.randint(256, size=(3, 224, 224), dtype=np.uint8) for name in camera_names},
        "prompt": "do something",
    }


def _random_observation_rtc(camera_names, state_dim) -> dict:
    return {
        **_random_observation(camera_names, state_dim),
        "action_prefix": np.ones((4, state_dim)),
        "delay": np.array(4),
    }


class OpenpiClient:
    def __init__(
        self,
        host: str,
        port: int,
        camera_names: list[str],
        state_dim: int,
    ) -> None:
        self.camera_names = list(camera_names)
        self.state_dim = state_dim

        # build client to connect server policy
        self.client = websocket_client_policy.WebsocketClientPolicy(host, port)

    def _build_observation(self, payload) -> dict:
        # payload["images"] maps camera name -> HWC image, e.g. cam_high/cam_left_wrist/cam_right_wrist
        # for the dual robot or cam_side/cam_wrist for the single robot
        images = {}
        for name in self.camera_names:
            img = image_tools.convert_to_uint8(image_tools.resize_with_pad(payload["images"][name], 224, 224))
            images[name] = img.transpose(2, 0, 1)

        observation = {
            "state": payload["state"],
            "images": images,
            "prompt": payload["instruction"],
        }

        if "action_prefix" in payload and payload["action_prefix"] is not None:
            observation["action_prefix"] = payload["action_prefix"]
            observation["delay"] = payload["delay"]

        return observation

    def predict_action(self, payload) -> np.ndarray:
        observation = self._build_observation(payload)
        response = self.client.infer(observation)
        return response["actions"]

    def predict_action_streaming(self, payload, on_actions_ready=None) -> np.ndarray:
        """Streaming prediction — calls *on_actions_ready(step, indices, actions)*
        for each group of action indices that finish denoising early.

        Returns the full action chunk once inference completes.
        """
        observation = self._build_observation(payload)
        response = self.client.infer_streaming(observation, on_actions_ready=on_actions_ready)
        if response is not None:
            return response["actions"]

    def warmup(self, rtc: bool = False, streaming: bool = False) -> None:
        observation = _random_observation(self.camera_names, self.state_dim)
        observation_rtc = _random_observation_rtc(self.camera_names, self.state_dim)
        if streaming:
            self.client.infer_streaming(observation)
            if rtc:
                self.client.infer_streaming(observation_rtc)
        else:
            self.client.infer(observation)
            if rtc:
                self.client.infer(observation_rtc)
