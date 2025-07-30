import datetime
import logging
import threading
import traceback
from typing import List, Optional

import numpy as np

from .config import DEFAULT_JOINT


class MockJakaRobot:
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
        self.mock_pose = [0.0] * 6
        self.mock_joint = DEFAULT_JOINT
        self.enabled = False
        self.powered_on = False
        self.in_servomove = False

    def __del__(self):
        self.leave_servo_mode()
        self.disable()
        self.stop()
        self.logger.info("Mock Robot deleted")

    def power_on(self) -> None:
        self.powered_on = True
        self.logger.info("Mock: power_on called")

    def enable_robot(self) -> None:
        self.enabled = True
        self.logger.info("Mock: enable_robot called")

    def start(self) -> None:
        self.logger.info("Mock: start called")
        self.powered_on = True

    def enable(self) -> None:
        self.logger.info("Mock: enable called")
        self.enabled = True

    def move_pose_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        self.move_pose(pose)
        self.logger.info("Mock: move_pose_until_completion called")
        return True

    def move_joint_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        self.move_joint(pose)
        self.logger.info("Mock: move_joint_until_completion called")
        return True

    def move_pose(self, pose) -> None:
        self.mock_pose = pose
        self.logger.info(f"Mock: move_pose called with {pose}")

    def move_joint(self, joint) -> None:
        self.mock_joint = joint
        self.logger.info(f"Mock: move_joint called with {joint}")

    def get_current_pose(self) -> List[float]:
        self.logger.info("Mock: get_current_pose called")
        return self.mock_pose

    def get_current_joint(self) -> List[float]:
        self.logger.info("Mock: get_current_joint called")
        return self.mock_joint

    def enter_servo_mode(self) -> None:
        self.in_servomove = True
        self.logger.info("Mock: enter_servo_mode called")

    def move_joint_servo(self, pose) -> None:
        self.mock_joint = (np.array(self.mock_joint) + np.array(pose)).tolist()
        self.logger.info(f"Mock: move_joint_servo called with {pose}")

    def leave_servo_mode(self) -> None:
        self.in_servomove = False
        self.logger.info("Mock: leave_servo_mode called")

    def disable(self) -> None:
        self.enabled = False
        self.logger.info("Mock: disable called")

    def stop(self) -> None:
        self.powered_on = False
        self.logger.info("Mock: stop called")

    def clear_error(self) -> None:
        self.logger.info("Mock: clear_error called")

    def is_powered_on(self) -> bool:
        self.logger.info("Mock: is_powered_on called")
        return self.powered_on

    def is_enabled(self) -> bool:
        self.logger.info("Mock: is_enabled called")
        return self.enabled

    def is_in_servomove(self) -> bool:
        self.logger.info("Mock: is_in_servomove called")
        return self.in_servomove
    
    def format_error(self, e: Exception) -> str:
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s

    def get_cur_error_info_all(self):
        self.logger.info("Mock: get_cur_error_info_all called")
        return []
    
    def are_all_errors_stateless(self, errors):
        self.logger.info("Mock: are_all_errors_stateless called")
        return False

    def recover_automatic_enable(self):
        self.logger.info("Mock: recover_automatic_enable called")
        self.clear_error()
        self.enable()
        return self.is_enabled()

    def jog_joint(self, joint: int, direction: float) -> None:
        joints = self.get_current_joint()
        joints = np.asarray(joints)
        joints[joint] += direction
        self.move_joint(joints.tolist())
        self.logger.info(f"Mock: jog_joint called for joint {joint} direction {direction}")

    def jog_tcp(self, axis: int, direction: float) -> None:
        poses = self.get_current_pose()
        poses = np.asarray(poses)
        poses[axis] += direction
        self.move_pose(poses.tolist())
        self.logger.info(f"Mock: jog_tcp called for axis {axis} direction {direction}")

    def emergency_stop_status(self) -> bool:
        self.logger.info("Mock: emergency_stop_status called")
        return False

    def take_arm(self) -> None:
        self.logger.info("Mock: take_arm called")
        pass


class MockJakaRobotFeedback:
    def __init__(
        self,
        name="mock_jaka_zu_5s_feedback",
        ip_feed: str = "10.5.5.100",
        port_feed: int = 10000,
        save_feed: bool = False,
        save_feed_path: Optional[str] = None,
        logger: Optional[logging.Logger] = None,
    ):
        if logger is None:
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logger
        self.name = name
        self.save_feed_fd = None
        if save_feed:
            if save_feed_path is None:
                save_feed_path = \
                    datetime.datetime.now().strftime("%Y%m%d%H%M%S") + \
                    "_feed.jsonl"
            self.save_feed_fd = open(save_feed_path, "w")
        self.__Lock = threading.Lock()

        self.mock_pose = [0.0] * 6
        self.mock_joint = DEFAULT_JOINT
        self.mock_feed = {
            "errcode": 0,
            "errmsg": "",
            "powered_on": True,
            "enabled": True,
            "paused": False,
            "on_soft_limit": False,
            "emergency_stop": False,
            "protective_stop": False,
            "actual_position": self.mock_pose,
            "joint_actual_position": self.mock_joint,
            "torqsensor": [[0]*6, [0]*6, [0]*6],
        }

    def __del__(self):
        if self.save_feed_fd is not None:
            self.save_feed_fd.close()
            self.save_feed_fd = None

    def start(self):
        self.logger.info("Mock: start called")
        self.latest_feed = self.mock_feed

    def _on_feed(self, data):
        self.logger.info(f"Mock: _on_feed called with {data}")
        self.latest_feed = data

    def _get_error_related_feedback(self, data):
        error_related_feedback = {
            "errcode": data["errcode"],
            "errmsg": data["errmsg"],
            "powered_on": data["powered_on"],
            "enabled": data["enabled"],
            "paused": data["paused"],
            "on_soft_limit": data["on_soft_limit"],
            "emergency_stop": data["emergency_stop"],
            "protective_stop": data["protective_stop"],
        }
        return error_related_feedback

    def is_powered_on_feed(self) -> bool:
        with self.__Lock:
            return self.latest_feed["powered_on"]

    def is_enabled_feed(self) -> bool:
        with self.__Lock:
            return self.latest_feed["enabled"]

    def is_emergency_stop_feed(self) -> bool:
        with self.__Lock:
            return self.latest_feed["emergency_stop"]

    def is_protective_stop_feed(self) -> bool:
        with self.__Lock:
            return self.latest_feed["protective_stop"]

    def get_current_pose_feed(self) -> List[float]:
        with self.__Lock:
            return self.latest_feed["actual_position"]

    def get_current_joint_feed(self) -> List[float]:
        with self.__Lock:
            return self.latest_feed["joint_actual_position"]

    def ForceValue_feed(self) -> List[float]:
        with self.__Lock:
            torqsensor = self.latest_feed["torqsensor"]
            return torqsensor[1][2]

    def format_error(self, e: Exception) -> str:
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s

    def get_cur_error_info_all(self) -> List[dict]:
        with self.__Lock:
            data = self.latest_feed
            errcode = data["errcode"]
            is_error = str(errcode) != "0"
            if is_error:
                error_related_feedback = self._get_error_related_feedback(data)
                return [error_related_feedback]
            else:
                return []

    def are_all_errors_stateless(self, errors: List[dict]) -> bool:
        return all(not error["emergency_stop"] for error in errors)
