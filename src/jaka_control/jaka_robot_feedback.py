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
        save_feed=False,
        save_feed_path="jaka_feed.jsonl",
        n_latest_errors: int = 5,
        ip_feed: str = "10.5.5.100",
        port_feed: int = 10000,
        logger: Optional[logging.Logger] = None,
    ):
        if logger is None:
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logger
        self.name = name
        self.client_feed = RCFeedBack(ip=ip_feed, port=port_feed)
        self.save_feed_fd = None
        self.n_latest_errors = n_latest_errors
        self.latest_errors = []
        self.latest_feed = None
        if save_feed:
            self.save_feed_fd = open(save_feed_path, "w")

    def __del__(self):
        if self.save_feed_fd is not None:
            self.save_feed_fd.close()
            self.save_feed_fd = None

    def start(self):
        self.logger.info("start")
        self.client_feed.login()
        self.client_feed.set_callback(self._on_feed)

    def _on_feed(self, data):
        if self.save_feed_fd is not None:
            self.save_feed_fd.write(json.dumps(data) + "\n")
        self.latest_feed = data
        errcode = data["errcode"]
        errmsg = data["errmsg"]
        protective_stop = data["protective_stop"]
        if protective_stop == 0:
            self.latest_errors = []
        else:
            self.logger.warning(("Robot error code and message", errcode, errmsg))
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

    def get_latest_error(self):
        return self.latest_errors

    def is_powered_on_feed(self):
        if self.latest_feed is None:
            return None
        return self.latest_feed["powered_on"]

    def is_enabled_feed(self):
        if self.latest_feed is None:
            return None
        return self.latest_feed["enabled"]

    def is_emergency_stop_feed(self):
        if self.latest_feed is None:
            return None
        return self.latest_feed["emergency_stop"]
        
    def is_protective_stop_feed(self):
        if self.latest_feed is None:
            return None
        return self.latest_feed["protective_stop"]

    def get_current_pose_feed(self) -> List[float]:
        if self.latest_feed is None:
            return None
        return self.latest_feed["actual_position"]

    def get_current_joint_feed(self) -> List[float]:
        if self.latest_feed is None:
            return None
        return self.latest_feed["joint_actual_position"]

    def ForceValue_feed(self) -> List[float]:
        if self.latest_feed is None:
            return None
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
        # HACK
        return False
