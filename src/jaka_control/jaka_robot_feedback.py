import datetime
import threading
import traceback
from typing import List, Optional

import json
import logging
import time

import numpy as np

from .jkrc_feedback import RCFeedBack


class JakaRobotFeedback:
    def __init__(
        self,
        name="jaka_zu_5s_feedback",
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
        self.client_feed = RCFeedBack(ip=ip_feed, port=port_feed)
        self.save_feed_fd = None
        if save_feed:
            if save_feed_path is None:
                save_feed_path = \
                    datetime.datetime.now().strftime("%Y%m%d%H%M%S") + \
                    "_feed.jsonl"
            self.save_feed_fd = open(save_feed_path, "w")
        self.__Lock = threading.Lock()

    def __del__(self):
        if self.save_feed_fd is not None:
            self.save_feed_fd.close()
            self.save_feed_fd = None

    def start(self):
        self.logger.info("start")
        self.latest_feed = None
        self.client_feed.login()
        self.client_feed.set_callback(self._on_feed)
        # 最初のフィードが来るまで待つ
        while True:
            with self.__Lock:
                if self.latest_feed is not None:
                    return
            time.sleep(0.008)

    def _on_feed(self, data):
        if self.save_feed_fd is not None:
            self.save_feed_fd.write(json.dumps(data) + "\n")
        with self.__Lock:
            self.latest_feed = data
        errcode = data["errcode"]
        is_error = str(errcode) != "0"
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
        """トルクセンサの実際の力 (6次元)"""
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
            error_related_feedback = self._get_error_related_feedback(data)
            return [error_related_feedback]

    def are_all_errors_stateless(self, errors: List[dict]) -> bool:
        return all(not error["emergency_stop"] for error in errors)


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
        self.mock_joint = [0.0] * 6
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
            error_related_feedback = self._get_error_related_feedback(data)
            return [error_related_feedback]

    def are_all_errors_stateless(self, errors: List[dict]) -> bool:
        return all(not error["emergency_stop"] for error in errors)
