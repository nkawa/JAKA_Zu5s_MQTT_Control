# Jaka の状態をモニタリングする

from typing import Literal
from jaka_robot import JakaRobot
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

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "10.5.5.10")
ROBOT_UUID = os.getenv("ROBOT_UUID","no-uuid")
MQTT_SERVER = os.getenv("MQTT_SERVER", "127.0.0.1")
MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")+"/"+ROBOT_UUID
MQTT_FORMAT = os.getenv("MQTT_FORMAT", "UR-realtime-control-MQTT")
MQTT_MODE = os.getenv("MQTT_MODE", "local")
SAVE = os.getenv("SAVE", "false") == "true"

# 基本的に運用時には固定するパラメータ
t_intv = 0.008
save_state = SAVE
if save_state:
    save_path = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + "_status.jsonl"
    f = open(save_path, "w")

class Jaka_MON:
    def __init__(self):
        pass

    def init_robot(self):
        self.robot = JakaRobot(
            ip_feed=ROBOT_IP,
            disable_move=True,
        )
        self.robot.start()

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
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)

    def on_connect(self,client, userdata, flag, rc,proc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示

    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def connect_mqtt(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始

    def monitor_start(self):
        last = 0
        last_error_monitored = 0
        while True:
            now = time.time()
            if last == 0:
                last = now
            if last_error_monitored == 0:
                last_error_monitored = now

            # TCP姿勢
            actual_tcp_pose = self.robot.get_current_pose_feed()
            # 関節
            actual_joint = self.robot.get_current_joint_feed()
            if MQTT_FORMAT == 'UR-realtime-control-MQTT':        
                joints = ['j1','j2','j3','j4','j5','j6']
                actual_joint_js = {
                    k: v for k, v in zip(joints, actual_joint)}
            elif MQTT_FORMAT == 'Jaka-Control-IK':
                # 7要素送る必要があるのでダミーの[0]を追加
                actual_joint_js = {"joints": list(actual_joint) + [0]}
                # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
                actual_joint_js["joints"][4] = actual_joint_js["joints"][4] - 90
            else:
                raise ValueError

            # 型: 整数、単位: ms
            time_ms = int(now * 1000)
            actual_joint_js["time"] = time_ms
            # [X, Y, Z, RX, RY, RZ]: センサ値の力[N]とモーメント[Nm]
            forces = self.robot.ForceValue_feed()
            actual_joint_js["forces"] = forces

            # エラーなしまたは制御プロセスでエラー対応成功
            if self.pose[14] == 0:
                error = {}
            # 制御プロセスでエラー検出
            elif self.pose[14] == 1:
                # 現在のエラー
                errors = self.robot.get_latest_error()
                error = {"errors": errors}
                # 自動復帰可能エラー
                # TODO: 開発中。まずはどのようなエラーが出るかcontrolで見る
                if True:
                    self.pose[14] = 2
                    error["auto_recoverable"] = True
                # 復帰にユーザーの対応を求めるエラー
                else:
                    self.pose[14] = 3
                    error["auto_recoverable"] = False
            # 上以外は対応中のエラーをerrorsとして出力しつづける
            if not error:
                actual_joint_js["error"] = error

            self.pose[:len(actual_joint)] = actual_joint

            if now-last > 0.3:
                self.client.publish(MQTT_ROBOT_STATE_TOPIC,
                                    json.dumps(actual_joint_js))
                last = now

            if save_state:
                datum = dict(
                    kind="state",
                    joint=actual_joint,
                    pose=actual_tcp_pose,
                    force=forces,
                    error=error,
                    time=now,
                )
                js = json.dumps(datum, ensure_ascii=False)
                f.write(js + "\n")

            t_elapsed = time.time() - now
            t_wait = t_intv - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("jaka")
        self.pose = np.ndarray((16,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.init_realtime()
        self.init_robot()
        self.connect_mqtt()
        try:
            self.monitor_start()
        except KeyboardInterrupt:
            print("Stop! Jaka monitor")
            self.robot.disable()
            self.robot.stop()

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
