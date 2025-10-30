# Implementation of Piper robot for LeRobot
import logging

from dataclasses import dataclass, field
from typing import Any

from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import Robot, RobotConfig

from .piper_sdk_interface import PiperSDKInterface

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
)
logger = logging.getLogger(__name__)


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
        self.config = config

        self.sdk = PiperSDKInterface(port="can0")
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
            "elbow_flex":    (-95.0,   95.0),
            "wrist_flex":    (-95.0,  95.0),
            "wrist_yaw":     (-95.0,  95.0),
            "wrist_roll":    (-24.0,  47.0),
            "gripper":       (  0.0, 100.0),
        }
        
        
        # self.bus = DynamixelMotorsBus(
        #     port=self.config.port,
        #     motors={
        #         "gripper": Motor(7, "xl330-m288", MotorNormMode.RANGE_0_100),
        #     },
        #     calibration=self.calibration,
        # )


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
        # return self.bus.is_connected
        pass
    
    def connect(self, calibrate: bool = True) -> None:
        for cam in self.cameras.values():
            cam.connect()
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()
        self.configure()
        logger.info(f"{self} connected.")


    # def connect(self, calibrate: bool = True) -> None:
    #     # Already connected in SDK init
    #     for cam in self.cameras.values():
    #         cam.connect()
    #     self.configure()

    def disconnect(self) -> None:
        
        input("Press Enter to home robot...")
        # self.sdk.home_ready()

        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # self.bus.disconnect(True)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
        
    def home(self) -> None:
        self.sdk.home()
        
    def home_ready(self) -> None:
        self.sdk.home_ready()

    @property
    def is_calibrated(self) -> bool:
        pass
        # return self.bus.is_calibrated
 
    def calibrate(self) -> None:
        pass
        # self.bus.disable_torque()
        # if self.calibration:
        #     # Calibration file exists, ask user whether to use it or run new calibration
        #     user_input = input(
        #         f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
        #     )
        #     if user_input.strip().lower() != "c":
        #         logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
        #         self.bus.write_calibration(self.calibration)
        #         return
            
        # logger.info(f"\nRunning calibration of {self}")
        # for motor in self.bus.motors:
        #     # use current based position for gripper
        #     self.bus.write("Operating_Mode", motor, OperatingMode.CURRENT_POSITION.value)

        # # self.bus.write("Drive_Mode", "elbow_flex", DriveMode.INVERTED.value)
        # drive_modes = {motor: 0 for motor in self.bus.motors}

        # input(f"Move {self} to the middle of its range of motion and press ENTER....")
        # homing_offsets = self.bus.set_half_turn_homings()

        # # full_turn_motors = ["shoulder_pan", "wrist_roll"]
        # unknown_range_motors = [motor for motor in self.bus.motors]
        # # print(
        # #     f"Move all joints except {full_turn_motors} sequentially through their "
        # #     "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        # # )
        # range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)


        # self.calibration = {}
        # for motor, m in self.bus.motors.items():
        #     self.calibration[motor] = MotorCalibration(
        #         id=m.id,
        #         drive_mode=drive_modes[motor],
        #         homing_offset=homing_offsets[motor],
        #         range_min=range_mins[motor],
        #         range_max=range_maxes[motor],
        #     )

        # self.bus.write_calibration(self.calibration)
        # self._save_calibration()
        # logger.info(f"Calibration saved to {self.calibration_fpath}")
        
    def configure(self) -> None:
        pass
        # with self.bus.torque_disabled():
        #     self.bus.configure_motors()
        #     for motor in self.bus.motors:
        #         if motor != "gripper":
        #             self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)
        #     self.bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value)


    def get_observation(self) -> dict[str, Any]:
        obs_dict = self.sdk.get_status()
        
        for motor, val in self.bus.sync_read("Present_Position").items():
            obs_dict[motor] = val

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()
            
            
        return obs_dict
    
    # def setup_motors(self) -> None:
    #     for motor in reversed(self.bus.motors):
    #         input(f"Connect the controller board to the '{motor}' motor only and press enter.")
    #         self.bus.setup_motor(motor)
    #         print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

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
        # still sending joint 6 position,
        # print(positions[6])
        
        #invert
        self.send_gripper_action(100-positions[6])
        return action


    def send_gripper_action(self, goal_pos: float) -> None:
        pass
        # if not self.is_connected:
        #     raise DeviceNotConnectedError(f"{self} is not connected.")
        
        # return self.bus.sync_write("Goal_Position", goal_pos)


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