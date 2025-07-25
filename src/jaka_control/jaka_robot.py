from faulthandler import is_enabled
import traceback
from typing import List, Optional

import json
import logging
import time

import numpy as np

from .jkrc import RC


class JakaRobotError(Exception):
    pass


class JakaRobot:
    def __init__(
        self,
        name="jaka_zu_5s",
        ip_move: str = "10.5.5.100",
        port_move: int = 10001,
        logger: Optional[logging.Logger] = None,
    ):
        if logger is None:
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logger
        self.name = name
        self.client_move = RC(ip=ip_move, port=port_move)

    def __del__(self):
        self.leave_servo_mode()
        self.disable()
        self.stop()
        self.logger.info("Robot deleted")

    def power_on(self) -> None:
        res = self.client_move.power_on()
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def enable_robot(self) -> None:
        res = self.client_move.enable_robot()
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def start(self) -> None:
        self.logger.info("start")
        self.client_move.login()
        cnt = 0
        if not self.is_powered_on():
            self.power_on()
            while not self.is_powered_on():
                time.sleep(1)
                cnt += 1
                if cnt > 30:
                    raise JakaRobotError(
                        "Failed to power on the robot in 30 seconds.")

    def enable(self) -> None:
        self.logger.info("enable")
        cnt = 0
        if not self.is_enabled():
            self.enable_robot()
            while not self.is_enabled():
                time.sleep(1)
                cnt += 1
                if cnt > 30:
                    raise JakaRobotError(
                        "Failed to enable the robot in 30 seconds.")

    def move_pose_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        self.move_pose(pose)
        if precisions is None:
            precisions = [1, 1, 1, 1, 1, 1]
        precisions = np.asarray(precisions)
        t_start = time.time()
        while True:
            current_pose = self.get_current_pose()
            diff = np.abs(np.asarray(current_pose) - np.asarray(pose))
            if np.all(diff < precisions):
                done = True
                break
            time.sleep(check_interval)
            if time.time() - t_start > timeout:
                self.logger.info("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_joint_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        self.move_joint(pose)
        if precisions is None:
            precisions = [1, 1, 1, 1, 1, 1]
        precisions = np.asarray(precisions)
        t_start = time.time()
        while True:
            current_pose = self.get_current_joint()
            diff = np.abs(np.asarray(current_pose) - np.asarray(pose))
            if np.all(diff < precisions):
                done = True
                break
            time.sleep(check_interval)
            if time.time() - t_start > timeout:
                self.logger.info("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_pose(self, pose) -> None:
        # absolute pose move
        res = self.client_move.end_move(pose, speed=10, accel=10)
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def move_joint(self, joint) -> None:
        # absolute joint move
        res = self.client_move.joint_move_with_acc(joint, 0, speed=10, accel=10)
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def get_current_pose(self) -> List[float]:
        res = self.client_move.get_tcp_position()
        if res[0] != 0:
            raise JakaRobotError(res[1])
        return res[1]

    def get_current_joint(self) -> List[float]:
        res = self.client_move.get_joint_position()
        if res[0] != 0:
            raise JakaRobotError(res[1])
        return res[1]

    def enter_servo_mode(self) -> None:
        res = self.client_move.servo_move_enable(True)
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def move_joint_servo(self, pose) -> None:
        """differential joint servo move"""
        res = self.client_move.servo_j(pose, 1)
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def leave_servo_mode(self):
        res = self.client_move.servo_move_enable(False)
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def disable(self) -> None:
        res = self.client_move.disable_robot()
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def stop(self):
        self.client_move.logout()

    def clear_error(self) -> None:
        res = self.client_move.clear_error()
        if res[0] != 0:
            raise JakaRobotError(res[1])

    def get_joint_position(self):
        ec, joint_pos = self.client_move.get_joint_position()
        if ec == 0:
            return joint_pos
        else:
            return None

    def is_powered_on(self) -> bool:
        res = self.client_move.get_robot_state()
        if res[0] != 0:
            raise JakaRobotError(res[1])
        return res[1] == 1

    def is_enabled(self) -> bool:
        res = self.client_move.get_robot_state()
        if res[0] == 0:
            raise JakaRobotError(res[1])
        return res[2] == 1

    def is_in_servomove(self) -> bool:
        res = self.client_move.is_in_servomove()
        if res[0] != 0:
            raise JakaRobotError(res[1])
        return res[1]
    
    def format_error(self, e: Exception) -> str:
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s

    def get_cur_error_info_all(self):
        # HACK
        return []
    
    def are_all_errors_stateless(self, errors):
        # HACK
        return False

    def recover_automatic_enable(self):
        self.clear_error()
        self.enable()
        return self.is_enabled()

    def jog_joint(self, joint: int, direction: float) -> None:
        joints = self.get_current_joint()
        joints = np.asarray(joints)
        joints[joint] += direction
        self.move_joint(joints.tolist())

    def jog_tcp(self, axis: int, direction: float) -> None:
        poses = self.get_current_pose()
        poses = np.asarray(poses)
        poses[axis] += direction
        self.move_pose(poses.tolist())

    def emergency_stop_status(self) -> bool:
        res = self.client_move.get_robot_state()
        if res[0] != 0:
            raise JakaRobotError(res[1])
        return res[1] == 1

    def take_arm(self) -> None:
        """Not required in Jaka"""
        # HACK
        pass
