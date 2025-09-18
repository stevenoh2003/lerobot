# Implementation of Piper robot for LeRobot

from dataclasses import dataclass, field
from typing import Any

from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import Robot, RobotConfig

from .piper_sdk_interface import PiperSDKInterface


@RobotConfig.register_subclass("piper")
@dataclass
class PiperConfig(RobotConfig):
    port: str
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_1": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )


class Piper(Robot):
    config_class = PiperConfig
    name = "piper"

    def __init__(self, config: PiperConfig):
        super().__init__(config)
        self.sdk = PiperSDKInterface(port=config.port)
        self.cameras = make_cameras_from_configs(config.cameras)
        
        self.joint_order = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_yaw",
            "wrist_roll",
            "gripper",
        ]

        # Per-joint normalized limits (edit as needed)
        self.joint_limits = {
            "shoulder_pan":  (-95.0,  95.0),
            "shoulder_lift": (-95.0,  95.0),
            "elbow_flex":    (-95.0,   95.0),  # example asymmetric limit
            "wrist_flex":    (-95.0,  95.0),
            "wrist_yaw":     (-95.0,  95.0),
            "wrist_roll":    (-24.0,  47.0),
            "gripper":       (  0.0, 100.0),
        }

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"joint_{i}.pos": float for i in range(7)}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        # Assume always connected after SDK init
        return True

    def connect(self, calibrate: bool = True) -> None:
        # Already connected in SDK init
        for cam in self.cameras.values():
            cam.connect()
        self.configure()

    def disconnect(self) -> None:
        self.sdk.home()
        for cam in self.cameras.values():
            cam.disconnect()
        
    def home(self) -> None:
        self.sdk.home()

    @property
    def is_calibrated(self) -> bool:
        return True
 
    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        obs_dict = self.sdk.get_status()

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()
        return obs_dict

    # def clip_joint_angle_percentage(self, percentages: dict[str, float]) -> dict[str, float]:
    #     clipped = {}
    #     for joint, percent in percentages.items():
    #         min_angle = self.sdk.min_pos[int(joint.split("_")[-1])]
    #         max_angle = self.sdk.max_pos[int(joint.split("_")[-1])]
    #         clipped[joint] = min(max(percent, min_angle), max_angle)
    #     return clipped

    # def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
    #     # map the action from the leader to joints for the follower
    #     positions = [
    #         action.get("shoulder_pan.pos"),
    #         action.get("shoulder_lift.pos"),
    #         action.get("elbow_flex.pos"),
    #         action.get("wrist_flex.pos"),
    #         action.get("wrist_yaw.pos"),
    #         -action.get("wrist_roll.pos"),
    #         action.get("gripper.pos"),
    #     ]
        
    #     positions = self.clip_positions(positions, self.joint_limits)
    #     print("Clipped positions:", positions)

    #     self.sdk.set_joint_positions(positions)
    #     return action
    
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        # map the action from the leader to joints for the follower using generic joint names
        positions = [
            action.get("joint_0.pos"),  # shoulder_pan
            action.get("joint_1.pos"),  # shoulder_lift
            action.get("joint_2.pos"),  # elbow_flex
            action.get("joint_3.pos"),  # wrist_flex
            action.get("joint_4.pos"),  # wrist_yaw
            -action.get("joint_5.pos"), # wrist_roll (negated)
            action.get("joint_6.pos"),  # gripper
        ]
        
        positions = self.clip_positions(positions, self.joint_limits)

        self.sdk.set_joint_positions(positions)
        return action


    def clip_positions(self, positions: list[float], limits: dict[str, tuple[float, float]]) -> list[float]:
        """
        Clip each joint to its (min,max) from JOINT_LIMITS, defaulting to [-100, 100].
        Keeps the list interface expected by the SDK.
        """
        out: list[float] = []
        for i, v in enumerate(positions):
            name = self.joint_order[i] if i < len(self.joint_order) else f"joint_{i}"
            lo, hi = limits.get(name, (-100.0, 100.0))
            v = 0.0 if v is None else float(v)
            if v < lo: v = lo
            if v > hi: v = hi
            out.append(v)
        return out