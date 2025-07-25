# Jaka の状態をモニタリングする

import logging
from typing import Any, Dict, List
from paho.mqtt import client as mqtt

import datetime
import time

import os
import sys
import json
import psutil

import multiprocessing as mp

import numpy as np

from dotenv import load_dotenv

from .config import SHM_NAME, SHM_SIZE, T_INTV
from .jaka_robot_feedback import JakaRobotFeedback
from .tools import tool_infos, tool_classes

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))

MQTT_MODE = os.getenv("MQTT_MODE", "metawork")
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
ROBOT_UUID = os.getenv("ROBOT_UUID","jaka-zu-real")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","jaka-zu-real")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")+"/"+ROBOT_UUID
MQTT_FORMAT = os.getenv("MQTT_FORMAT", "Jaka-Control-IK")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
ROBOT_IP = os.getenv("ROBOT_IP", "10.5.5.10")
SAVE = os.getenv("SAVE", "true") == "true"

# 基本的に運用時には固定するパラメータ
save_state = SAVE
if save_state:
    save_path = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + "_status.jsonl"
    f = open(save_path, "w")

class Jaka_MON:
    def __init__(self):
        pass

    def init_robot(self):
        self.robot = JakaRobotFeedback(
            ip_feed=ROBOT_IP,
            logger=self.robot_logger,
        )
        self.robot.start()
        tool_id = int(os.environ["TOOL_ID"])
        self.find_and_setup_hand(tool_id)

    def find_and_setup_hand(self, tool_id):
        connected = False
        tool_info = self.get_tool_info(tool_infos, tool_id)
        name = tool_info["name"]
        hand = tool_classes[name]()
        if tool_id != -1:
            connected = hand.connect_and_setup()
            if not connected:
                raise ValueError(f"Failed to connect to hand: {name}")
        else:
            hand = None
        self.hand_name = name
        self.hand = hand
        self.tool_id = tool_id

    def reconnect_after_timeout(self, e: Exception) -> bool:
        # TODO
        return False

    def init_realtime(self):
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                self.logger.warning("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                self.logger.info("Process real-time priority set to: %u" % rt_app_priority)

    def on_connect(self,client, userdata, flag, rc,proc):
        self.logger.info("MQTT connected with result code: " + str(rc))  # 接続できた旨表示

    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            self.logger.warning("MQTT unexpected disconnection.")

    def connect_mqtt(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def monitor_start(self):
        last = 0
        last_error_monitored = 0
        is_in_tool_change = False
        is_put_down_box = False
        while True:
            now = time.time()
            if last == 0:
                last = now
            if last_error_monitored == 0:
                last_error_monitored = now

            actual_joint_js = {}

            # ツール番号
            tool_id = int(self.pose[23].copy())
            # 起動時（共有メモリが初期化されている）のみ初期値を使用
            if tool_id == 0:
                tool_id = self.tool_id
            # それ以外は制御プロセスからの値を使用
            else:
                self.tool_id = tool_id

            # ツールチェンジ
            status_tool_change = None
            next_tool_id = int(self.pose[17].copy())
            if next_tool_id != 0:
                if not is_in_tool_change:
                    is_in_tool_change = True
                    self.logger.info("Tool change started")
            else:
                if is_in_tool_change:
                    is_in_tool_change = False
                    status_tool_change = bool(self.pose[18] == 1)
                    self.logger.info("Tool change finished")
            if status_tool_change is not None:
                self.logger.info(f"Tool change status: {status_tool_change}")
                # 終了した場合のみキーを追加
                actual_joint_js["tool_change"] = status_tool_change
                # 成功した場合のみ接続
                if status_tool_change:
                    next_tool_info = self.get_tool_info(tool_infos, tool_id)
                    name = next_tool_info["name"]
                    hand = tool_classes[name]()
                    connected = hand.connect_and_setup()
                    if not connected:
                        self.logger.warning(
                            f"Failed to connect to hand: {name}")
                    self.hand_name = name
                    self.hand = hand

            # 棚の上の箱を作業台に置くデモ
            status_put_down_box = None
            put_down_box = self.pose[21].copy()
            if put_down_box != 0:
                if not is_put_down_box:
                    is_put_down_box = True
            else:
                if is_put_down_box:
                    is_put_down_box = False
                    status_put_down_box = bool(self.pose[22] == 1)
            # 終了した場合のみキーを追加
            if status_put_down_box is not None:
                actual_joint_js["put_down_box"] = status_put_down_box

            # TCP姿勢
            try:
                actual_tcp_pose = self.robot.get_current_pose_feed()
            except Exception as e:
                self.logger.error("Error in get_current_pose: ")
                self.logger.error(f"{self.robot.format_error(e)}")
                self.reconnect_after_timeout(e)
                actual_tcp_pose = None
            # 関節
            try:
                actual_joint = self.robot.get_current_joint_feed()
            except Exception as e:
                self.logger.error("Error in get_current_joint: ")
                self.logger.error(f"{self.robot.format_error(e)}")
                self.reconnect_after_timeout(e)
                actual_joint = None
            if actual_joint is not None:
                if MQTT_FORMAT == 'UR-realtime-control-MQTT':        
                    joints = ['j1','j2','j3','j4','j5','j6']
                    actual_joint_js.update({
                        k: v for k, v in zip(joints, actual_joint)})
                elif MQTT_FORMAT == 'Jaka-Control-IK':
                    # 7要素送る必要があるのでダミーの[0]を追加
                    actual_joint_js.update({"joints": list(actual_joint) + [0]})
                else:
                    raise ValueError
            
            # 型: 整数、単位: ms
            time_ms = int(now * 1000)
            actual_joint_js["time"] = time_ms
            # [X, Y, Z, RX, RY, RZ]: センサ値の力[N]とモーメント[Nm]
            try:
                forces = self.robot.ForceValue_feed()
            except Exception as e:
                self.logger.error("Error in ForceValue: ")
                self.logger.error(f"{self.robot.format_error(e)}")
                self.reconnect_after_timeout(e)
                forces = None
            if forces is not None:
                actual_joint_js["forces"] = forces

            actual_joint_js["tool_id"] = int(tool_id)
            # ツールチェンジ中はツールのセンサは使用しない
            if is_in_tool_change:
                width = None
                force = None
            else:
                # ツール依存の部分はまとめるべき
                if tool_id == -1:
                    width = None
                    force = None
                else:
                    if self.hand_name == "onrobot_2fg7":
                        try:
                            width = self.hand.get_ext_width()
                            force = self.hand.get_force()
                        except Exception as e:
                            self.logger.error("Error in onrobot_2fg7 hand: ")
                            self.logger.error(f"{self.robot.format_error(e)}")
                            width = None
                            force = None
                    elif self.hand_name == "onrobot_vgc10":
                        width = None
                        force = None
                    elif self.hand_name == "cutter":
                        width = None
                        force = None
                    elif self.hand_name == "plate_holder":
                        width = None
                        force = None

            # モータがONか
            try:
                enabled = self.robot.is_enabled_feed()
            except Exception as e:
                self.logger.warning(
                    "Somehow failed in checking if robot is enabled. "
                    "Enabled value may be incorrect.")
                self.logger.warning(f"{self.robot.format_error(e)}")
                self.reconnect_after_timeout(e)
                enabled = False
            actual_joint_js["enabled"] = enabled

            error = {}
            # スレーブモード中にエラー情報を取得しようとすると、
            # スレーブモードが切断される。
            # 本プロセスでロボットがスレーブモードでないと判断した直後に、
            # 別プロセスでスレーブモードに入る可能性があるので、
            # 通常モードの場合のみ呼び出す
            with self.slave_mode_lock:
                if self.pose[14] == 0:
                    actual_joint_js["servo_mode"] = False
                    try:
                        errors = self.robot.get_cur_error_info_all()
                    except Exception as e:
                        self.logger.error("Error in get_cur_error_info_all: ")
                        self.logger.error(f"{self.robot.format_error(e)}")
                        self.reconnect_after_timeout(e)
                        errors = []
                    # 制御プロセスのエラー検出と方法が違うので、
                    # 直後は状態プロセスでエラーが検出されないことがある
                    # その場合は次のループに検出を持ち越す
                    if len(errors) > 0:
                        error = {"errors": errors}
                        # 自動復帰可能エラー
                        try:
                            auto_recoverable = \
                                self.robot.are_all_errors_stateless(errors)
                            error["auto_recoverable"] = auto_recoverable
                        except Exception as e:
                            self.logger.error(
                                "Error in are_all_errors_stateless: ")
                            self.logger.error(f"{self.robot.format_error(e)}")
                            self.reconnect_after_timeout(e)
                            error["auto_recoverable"] = False
                else:
                    actual_joint_js["servo_mode"] = True
            if self.pose[15] == 0:
                actual_joint_js["mqtt_control"] = "OFF"
            else:
                actual_joint_js["mqtt_control"] = "ON"

            if error:
                actual_joint_js["error"] = error

            if actual_joint is not None:
                self.pose[:len(actual_joint)] = actual_joint
                self.pose[19] = 1

            if now-last > 0.3 or "tool_change" in actual_joint_js or "put_down_box" in actual_joint_js:
                jss = json.dumps(actual_joint_js)
                self.client.publish(MQTT_ROBOT_STATE_TOPIC, jss)
                with self.monitor_lock:
                    actual_joint_js["topic_type"] = "robot"
                    actual_joint_js["topic"] = MQTT_ROBOT_STATE_TOPIC
                    if actual_tcp_pose is not None:
                        actual_joint_js["poses"] = actual_tcp_pose
                    self.monitor_dict.clear()
                    self.monitor_dict.update(actual_joint_js)
                last = now

            if save_state:
                datum = dict(
                    kind="state",
                    joint=actual_joint,
                    pose=actual_tcp_pose,
                    width=width,
                    force=force,
                    forces=forces,
                    error=error,
                    time=now,
                    enabled=enabled,
                    # TypeError: Object of type float32 is not JSON
                    # serializableへの対応
                    tool_id=float(tool_id),
                )
                js = json.dumps(datum, ensure_ascii=False)
                f.write(js + "\n")

            t_elapsed = time.time() - now
            t_wait = T_INTV - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("MON")
        if log_queue is not None:
            handler = logging.handlers.QueueHandler(log_queue)
        else:
            handler = logging.StreamHandler()
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        self.robot_logger = logging.getLogger("MON-ROBOT")
        if log_queue is not None:
            handler = logging.handlers.QueueHandler(log_queue)
        else:
            handler = logging.StreamHandler()
        self.robot_logger.addHandler(handler)
        self.robot_logger.setLevel(logging.WARNING)

    def run_proc(self, monitor_dict, monitor_lock, slave_mode_lock, log_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.monitor_dict = monitor_dict
        self.monitor_lock = monitor_lock
        self.slave_mode_lock = slave_mode_lock

        self.init_realtime()
        self.init_robot()
        self.connect_mqtt()
        try:
            self.monitor_start()
        except KeyboardInterrupt:
            self.logger.info("Stop! Jaka Zu monitor")
            self.robot.disable()
            self.robot.stop()
        except Exception as e:
            self.logger.error("Error in monitor")
            self.logger.error(f"{self.robot.format_error(e)}")

if __name__ == '__main__':
    cp = Jaka_MON()
    cp.init_realtime()
    cp.init_robot()
    cp.connect_mqtt()

    try:
        cp.monitor_start()
    except KeyboardInterrupt:
        print("Monitor Main Stopped")
        cp.robot.disable()
        cp.robot.stop()
