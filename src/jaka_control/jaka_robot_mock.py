import datetime
import json
import logging
import multiprocessing as mp
import multiprocessing.shared_memory
import threading
import time
import traceback
from typing import List, Optional

import numpy as np

from .config import DEFAULT_JOINT


SHM_NAME = "mock_jaka"
SHM_SIZE = 32


class MockJakaRobotSharedMemoryManager:
    def __init__(self):
        sz = SHM_SIZE * np.dtype('float32').itemsize
        try:
            self.sm = mp.shared_memory.SharedMemory(create=True,size = sz, name=SHM_NAME)
        except FileExistsError:
            self.sm = mp.shared_memory.SharedMemory(size = sz, name=SHM_NAME)
        # self.arの要素の説明
        # [0:6]: 関節の状態値
        # [6:12]: 位置の状態値
        # [12]: ロボットの電源
        # [13]: モーターの電源
        # [14]: スレーブモードの状態
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.pose[:] = 0
        self.pose[0:6] = DEFAULT_JOINT  # 初期関節値を設定

    def _del_sm(self):
        self.sm.close()
        self.sm.unlink()

    def __del__(self):
        self._del_sm()


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
        self._init_sm()

    def _init_sm(self):
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)

    def _del_sm(self):
        self.sm.close()
        self.sm.unlink()

    def __del__(self):
        self.leave_servo_mode()
        self.disable()
        self.stop()
        self.logger.info("Mock Robot deleted")
        self._del_sm()

    def power_on(self) -> None:
        self.pose[12] = 1
        self.logger.info("Mock: power_on called")

    def enable_robot(self) -> None:
        self.pose[13] = 1
        self.logger.info("Mock: enable_robot called")

    def start(self) -> None:
        self.logger.info("Mock: start called")
        self.power_on()

    def enable(self) -> None:
        self.logger.info("Mock: enable called")
        self.enable_robot()

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
        self.pose[6:12] = pose
        self.logger.info(f"Mock: move_pose called with {pose}")

    def move_joint(self, joint) -> None:
        self.pose[0:6] = joint
        self.logger.info(f"Mock: move_joint called with {joint}")

    def get_current_pose(self) -> List[float]:
        self.logger.info("Mock: get_current_pose called")
        return self.pose[6:12].tolist()

    def get_current_joint(self) -> List[float]:
        self.logger.info("Mock: get_current_joint called")
        return self.pose[0:6].tolist()

    def enter_servo_mode(self) -> None:
        self.pose[14] = 1
        self.logger.info("Mock: enter_servo_mode called")

    def move_joint_servo(self, pose) -> None:
        self.pose[0:6] = (np.array(self.pose[0:6]) + np.array(pose)).tolist()
        # Stop logging because often called too frequently
        # self.logger.info(f"Mock: move_joint_servo called with {pose}")

    def leave_servo_mode(self) -> None:
        self.pose[14] = 0
        self.logger.info("Mock: leave_servo_mode called")

    def disable(self) -> None:
        self.pose[13] = 0
        self.logger.info("Mock: disable called")

    def stop(self) -> None:
        self.pose[12] = 0
        self.logger.info("Mock: stop called")

    def clear_error(self) -> None:
        self.logger.info("Mock: clear_error called")

    def is_powered_on(self) -> bool:
        self.logger.info("Mock: is_powered_on called")
        return bool(self.pose[12] == 1)

    def is_enabled(self) -> bool:
        self.logger.info("Mock: is_enabled called")
        return bool(self.pose[13] == 1) 

    def is_in_servomove(self) -> bool:
        self.logger.info("Mock: is_in_servomove called")
        return bool(self.pose[14] == 1)

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
        self._init_sm()

    def _init_sm(self):
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)

    def _del_sm(self):
        self.sm.close()
        self.sm.unlink()

    def __del__(self):
        if self.save_feed_fd is not None:
            self.save_feed_fd.close()
            self.save_feed_fd = None

    def recvFeedData(self):
        while True:
            # Simulate data receiving interval
            time.sleep(0.03)
            feed_data = {
                "errcode": 0,
                "errmsg": "",
                "powered_on": bool(self.pose[12] == 1),
                "enabled": bool(self.pose[13] == 1),
                "paused": False,
                "on_soft_limit": False,
                "emergency_stop": False,
                "protective_stop": False,
                "actual_position": self.pose[6:12].tolist(),
                "joint_actual_position": self.pose[0:6].tolist(),
                "torqsensor": [[0]*6, [0]*6, [0]*6],
            }
            feed_data["timestamp"] = time.perf_counter()
            self._on_feed(feed_data)

    def start(self):
        self.logger.info("Mock: start called")
        self.latest_feed = {}
        feed_thread = threading.Thread(target=self.recvFeedData)
        feed_thread.daemon = True
        feed_thread.start()
        # 最初のフィードが来るまで待つ
        while True:
            with self.__Lock:
                if self.latest_feed:
                    return
            time.sleep(0.008)

    def _on_feed(self, data):
        if self.save_feed_fd is not None:
            self.save_feed_fd.write(json.dumps(data) + "\n")
        with self.__Lock:
            self.latest_feed = data
        errcode = data["errcode"]
        is_error = str(errcode) not in ["0", "0x0"]
        if is_error:
            error_related_feedback = self._get_error_related_feedback(data)
            self.logger.error(
                f"Error in feedback: {error_related_feedback}")

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
            is_error = str(errcode) not in ["0", "0x0"]
            if is_error:
                error_related_feedback = self._get_error_related_feedback(data)
                return [error_related_feedback]
            else:
                return []

    def are_all_errors_stateless(self, errors: List[dict]) -> bool:
        return all(not error["emergency_stop"] for error in errors)
