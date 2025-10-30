# Piper SDK interface for LeRobot integration

import time
from typing import Any

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Is the piper_sdk installed: pip install piper_sdk")
    C_PiperInterface_V2 = None  # For type checking and docs


class PiperSDKInterface:
    def __init__(self, port: str = "can0"):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed. Please install it with `pip install piper_sdk`.")
        try:
            self.piper = C_PiperInterface_V2(port)
        except Exception as e:
            print(
                f"Failed to initialize Piper SDK: {e} Did you activate the can interface with `piper_sdk/can_activate.sh can0 1000000`"
            )
            self.piper = None
            return

        self.piper.ConnectPort()
 

        time.sleep(0.1)  # wait for connection to establish

        # # # reset the arm if it's not in idle state
        # print(self.piper.GetArmStatus().arm_status.motion_status)
        # # if self.piper.GetArmStatus().arm_status.motion_status != 0:
        # #     # self.piper.EmergencyStop(0x02)  # resume
        if self.piper.GetArmStatus().arm_status.ctrl_mode!=0x1:
            input("Press Enter to set control mode to 0x1")
            self.piper.MotionCtrl_1(emergency_stop=0x02, track_ctrl=0, grag_teach_ctrl=0)#恢复
            self.piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0, move_spd_rate_ctrl=0, is_mit_mode=0x00)#位置速度模式
            
            
        print(self.piper.GetArmStatus())
        
        # #     print(self.piper.GetArmStatus().arm_status.motion_status)
        print(self.piper.GetArmStatus().arm_status.motion_status)


        # # Set motion control to joint mode at 100% speed

        self.piper.EnableArm(7, 0x02)  # enable all joints
        time.sleep(0.1)

        self.piper.JointConfig(7, clear_err=0xAE)  # clear all joint errors
        self.piper.ModeCtrl(0x01, 0x01, 50, 0x00)

        # self.piper.MotionCtrl_2(0x01, 0x01, 70, 0x00)
        self.piper.JointMaxAccConfig(motor_num=1, max_joint_acc=500)
        self.piper.JointMaxAccConfig(motor_num=2, max_joint_acc=200)
        self.piper.JointMaxAccConfig(motor_num=3, max_joint_acc=200)
        self.piper.JointMaxAccConfig(motor_num=4, max_joint_acc=500)
        self.piper.JointMaxAccConfig(motor_num=5, max_joint_acc=500)
        self.piper.JointMaxAccConfig(motor_num=6, max_joint_acc=500)
        # self.piper.JointMaxAccConfig(max_joint_acc=50)
        
        print("Robot config", self.piper.GetAllMotorAngleLimitMaxSpd())
        input("Press Enter to continue...")
        ## Previous speed settings
        # self.piper.MotorMaxSpdSet(motor_num=4, max_joint_spd=300)
        # self.piper.MotorMaxSpdSet(motor_num=5, max_joint_spd=300)
        # self.piper.MotorMaxSpdSet(motor_num=6, max_joint_spd=300)
        
        
        # Improved speed settings
        self.piper.MotorMaxSpdSet(motor_num=4, max_joint_spd=1500) # was 300
        self.piper.MotorMaxSpdSet(motor_num=5, max_joint_spd=3000) # was 300
        self.piper.MotorMaxSpdSet(motor_num=6, max_joint_spd=3000) # was 300
        
        
        # # Get the min and max positions for each joint and gripper
        angel_status = self.piper.GetAllMotorAngleLimitMaxSpd()
        self.min_pos = [
            pos.min_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [0]
        self.max_pos = [
            pos.max_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [10]  # Gripper max position in mm


    def health_check_and_recover(self):
        """Detect CAN/mode issues and try to recover (SDK-version-safe)."""
        try:
            # 1) If SDK thread unhealthy, hard reconnect
            if not self.piper.isOk():
                try:
                    self.piper.DisconnectPort()
                except Exception:
                    pass
                self.piper.ConnectPort(can_init=True, piper_init=True, start_thread=True)
                time.sleep(0.05)
                self._ensure_mode_and_enable(speed_pct=60)
                return  # already recovered

            # 2) If CAN FPS looks dead/very low, soft recover
            fps = 0
            try:
                fps = self.piper.GetCanFps()
            except Exception:
                pass

            if fps <= 1:  # tweak threshold if needed
                # Soft recover: reassert mode & enable
                self._ensure_mode_and_enable(speed_pct=60)

            # 3) If motion_status not normal, resume & reassert mode
            st = self.piper.GetArmStatus()
            if getattr(st.arm_status, "motion_status", 0) != 0:
                self.piper.EmergencyStop(0x02)
                self._ensure_mode_and_enable(speed_pct=60)

        except Exception as e:
            print("Health check error:", e)


    def _recover_if_comm_lost(self):
        st = self.piper.GetArmStatus()
        flags = [
            st.arm_status.joint_1_comm_status,
            st.arm_status.joint_2_comm_status,
            st.arm_status.joint_3_comm_status,
            st.arm_status.joint_4_comm_status,
            st.arm_status.joint_5_comm_status,
            st.arm_status.joint_6_comm_status,
        ]
        if not any(flags):  # all False
            # Hard recover
            try:
                self.piper.DisconnectPort()
            except Exception:
                pass
            self.piper.ConnectPort(can_init=True, piper_init=True, start_thread=True)
            time.sleep(0.05)
            self._ensure_mode_and_enable(speed_pct=60)
  
    def _ensure_mode_and_enable(self, speed_pct: int = 60):
        """Ensure CAN command + MOVE_J and motors enabled."""
        st = self.piper.GetArmStatus()
        # Resume from estop/teach/failed state if needed
        if getattr(st.arm_status, "motion_status", 0) != 0:
            self.piper.EmergencyStop(0x02)
        # Reassert CAN command + MOVE_J @ speed_pct
        self.piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x01,
                            move_spd_rate_ctrl=speed_pct, is_mit_mode=0x00)
        # Enable all joints (7 = all)
        self.piper.EnableArm(7, 0x02)

    def set_joint_positions(self, positions):

        # positions: list of 7 floats, first 6 are joint and 7 is gripper position
        # positions are in -100% to 100% range, we need to map them on the min and max positions
        # so -100% is min_pos and 100% is max_pos
        scaled_positions = [
            self.min_pos[i] + (self.max_pos[i] - self.min_pos[i]) * (pos + 100) / 200
            for i, pos in enumerate(positions[:6])
        ]
        scaled_positions = [100.0 * pos for pos in scaled_positions]  # Adjust factor

        # the gripper is from 0 to 100% range
        scaled_positions.append(self.min_pos[6] + (self.max_pos[6] - self.min_pos[6]) * positions[6] / 100)
        scaled_positions[6] = int(scaled_positions[6] * 10000)  # Convert to mm

        # joint 0, 3 and 5 are inverted
        joint_0 = int(-scaled_positions[0])
        joint_1 = int(scaled_positions[1])
        joint_2 = int(scaled_positions[2])
        joint_3 = int(-scaled_positions[3])
        joint_4 = int(scaled_positions[4])
        joint_5 = int(-scaled_positions[5])
        joint_6 = int(scaled_positions[6])

        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(joint_6, 1000, 0x01, 0)

    def collision_msg(self):
        return self.piper.GetCrashProtectionLevelFeedback()

    def get_arm_status(self):
        return self.piper.GetArmStatus()
    
    def get_status(self) -> dict[str, Any]:
        joint_status = self.piper.GetArmJointMsgs()
        gripper = self.piper.GetArmGripperMsgs()

        joint_state = joint_status.joint_state
        obs_dict = {
            "joint_0.pos": joint_state.joint_1,
            "joint_1.pos": joint_state.joint_2,
            "joint_2.pos": joint_state.joint_3,
            "joint_3.pos": joint_state.joint_4,
            "joint_4.pos": joint_state.joint_5,
            "joint_5.pos": joint_state.joint_6,
        }
        obs_dict.update(
            {
                "joint_6.pos": gripper.gripper_state.grippers_angle,
            }
        )

        return obs_dict

    def home(self):
        self.piper.ModeCtrl(0x01, 0x01, 10, 0x00) # decrease speed for homing
        time.sleep(0.1)
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        self.piper.ModeCtrl(0x01, 0x01, 30, 0x00) # decrease speed for homing


    def home_ready(self):
        self.piper.ModeCtrl(0x01, 0x01, 10, 0x00) # decrease speed for homing
        time.sleep(0.1)
        self.piper.JointCtrl(0, 56179, -62415, 1625, 61228, 0)
        self.piper.ModeCtrl(0x01, 0x01, 30, 0x00) # decrease speed for homing
        