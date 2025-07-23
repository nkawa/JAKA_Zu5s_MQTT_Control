import traceback
from typing import List, Optional

import json
import logging
import time

import numpy as np

from .jkrc import RC, RCFeedBack


logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

class JakaRobot:
    def __init__(
        self,
        name="jaka_zu_5s",
        save_feed=False,
        save_feed_path="jaka_feed.jsonl",
        n_latest_errors: int = 5,
        ip_move: str = "10.5.5.100",
        port_move: int = 10001,
        ip_feed: str = "10.5.5.100",
        port_feed: int = 10000,
        disable_move: bool = False,
        disable_feed: bool = False,
    ):
        logger.info("__init__")
        self.name = name
        self.move = not disable_move
        self.feed = not disable_feed
        if self.move:
            self.client_move = RC(ip=ip_move, port=port_move)
            self.defPose = [50.67851, -120.33679, 452.7194, 166.39565, -18.02921, -130.30474]
            self.defJoint = [-179.024618, 40.256525, 107.558967, 144.281236, -94.064602, 44.242293]
        if self.feed:
            self.client_feed = RCFeedBack(ip=ip_feed, port=port_feed)
            self.save_feed_fd = None
            self.n_latest_errors = n_latest_errors
            self.latest_errors = []
            self.latest_feed = None
            if save_feed:
                self.save_feed_fd = open(save_feed_path, "w")

    def __del__(self):
        if self.feed:
            if self.save_feed_fd is not None:
                self.save_feed_fd.close()
                self.save_feed_fd = None

    def start(self):
        logger.info("start")
        if self.feed:
            self.client_feed.login()
            self.client_feed.set_callback(self._on_feed)
        if self.move:
            self.client_move.login()
            cnt = 10
            is_powered_on = False
            while True:
                try:
                    is_powered_on = self.is_powered_on()
                    break
                except ValueError:
                    cnt += 1
                    time.sleep(1)
                    if cnt == 10:
                        raise ValueError("Cannot start")
            if not is_powered_on:
                return self.client_move.power_on()
            else:
                logger.info("Already powered on")

    def _on_feed(self, data):
        if self.feed:
            if self.save_feed_fd is not None:
                self.save_feed_fd.write(json.dumps(data) + "\n")
            errcode = data["errcode"]
            errmsg = data["errmsg"]
            if errcode != "0x0":
                logger.warning(("Robot error code and message", errcode, errmsg))
                if len(self.latest_errors) >= self.n_latest_errors:
                    self.latest_errors = self.latest_errors[-(self.n_latest_errors - 1):]
                self.latest_errors += [{
                    "errcode": data["errcode"],
                    "errmsg": data["errmsg"],
                    "powered_on": data["powered_on"],
                    "enabled": data["enabled"],
                    "paused": data["paused"],
                    "on_soft_limit": data["on_soft_limit"],
                    "emergency_stop": data["emergency_stop"],
                    "protective_stop": data["protective_stop"],
                }]
            self.latest_feed = data

    def enable(self):
        logger.info("enable")
        if not self.is_enabled():
            return self.client_move.enable_robot()
        else:
            logger.info("Already enabled")

    def set_default_pose(self, pose):
        self.defPose = pose

    def get_default_pose(self):
        return self.defPose

    def get_default_joint(self):
        return self.defJoint

    def move_default_pose(self):
        pose = self.defPose
        self.move_pose(pose)

    def move_default_pose_until_completion(
        self,
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        pose = self.get_default_pose()
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
                logger.warning("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_default_joint_until_completion(
        self,
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        pose = self.get_default_joint()
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
                logger.warning("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_pose_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> None:
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
                logger.info("Timeout before reaching destination.")
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
    ) -> None:
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
                logger.info("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_pose(self, pose) -> int:
        logger.debug(("move_pose", pose))
        return self.client_move.end_move(pose, 10, 10)[0]

    def move_joint(self, joint) -> int:
        logger.debug(("move_joint", joint))
        return self.client_move.joint_move_extend(joint, 0, speed=10, accel=10)[0]

    def get_current_pose(self) -> List[float]:
        ec, pose = self.client_move.get_tcp_position()
        logger.debug(("get_current_pose", pose))
        return pose

    def get_current_joint(self) -> List[float]:
        ec, joint = self.client_move.get_joint_position()
        logger.debug(("get_joint_position", joint))
        return joint

    def enter_servo_mode(self):
        logger.info("enter_servo_mode")
        return self.client_move.servo_move_enable(True)

    def move_pose_servo(self, pose):
        ec = self.client_move.servo_p(pose, 1)[0]
        is_success = ec == 0
        if not is_success:
            logger.warning(("move_pose_servo error code: ", ec))
        return is_success

    def move_joint_servo(self, pose):
        return self.client_move.servo_j(pose, 1)

    def leave_servo_mode(self):
        self.client_move.servo_move_enable(False)[0]

    def disable(self):
        if self.move:
            self.client_move.disable_robot()

    def stop(self):
        if self.move:
            self.client_move.logout()

    def get_suggested_servo_interval(self):
        return 0.001

    def get_latest_error(self):
        return self.latest_errors

    def clear_error(self):
        self.client_move.clear_error()

    def get_joint_position(self):
        ec, joint_pos = self.client_move.get_joint_position()
        if ec == 0:
            return joint_pos
        else:
            return None

    def is_powered_on(self):
        ret = self.client_move.get_robot_state()
        return ret["power"] == "powered_on"

    def is_enabled(self):
        ret = self.client_move.get_robot_state()
        return ret["enable"] == "robot_enabled"
    
    def is_enabled_feed(self):
        if self.latest_feed is not None:
            return self.latest_feed["enabled"]

    def is_protective_stop(self):
        (ec, ps) = self.client_move.protective_stop_status()
        return ps

    def is_powered_on_feed(self):
        if self.latest_feed is None:
            raise ValueError
        return self.latest_feed["powered_on"]

    def is_enabled_feed(self):
        if self.latest_feed is None:
            raise ValueError
        return self.latest_feed["enabled"]

    def is_emergency_stop_feed(self):
        if self.latest_feed is None:
            raise ValueError
        return self.latest_feed["emergency_stop"]
        
    def is_protective_stop_feed(self):
        if self.latest_feed is None:
            raise ValueError
        return self.latest_feed["protective_stop"]

    def is_in_servomove(self) -> bool:
        ec, in_servomove = self.client_move.is_in_servomove()
        if ec != 0:
            raise ValueError
        return in_servomove

    def get_current_pose_feed(self) -> List[float]:
        return self.latest_feed["actual_position"]

    def get_current_joint_feed(self) -> List[float]:
        return self.latest_feed["joint_actual_position"]

    def ForceValue_feed(self) -> List[float]:
        """トルクセンサの実際の力 (6次元)"""
        torqsensor = self.latest_feed["torqsensor"]
        return torqsensor[1][2]
    
    def format_error(self, e: Exception) -> str:
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s

    def get_cur_error_info_all(self):
        return self.get_latest_error()
    
    def are_all_errors_stateless(self, errors):
        # TODO
        return False

    def recover_automatic_enable(self):
        # TODO
        self.clear_error()
        self.enable()

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
