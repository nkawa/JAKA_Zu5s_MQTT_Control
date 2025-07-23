"""JAKA Zu 5sのPython APIの自前実装。

JAKA Zu 5sのPython APIのドキュメントは取得できたが、コード(jkrc.py)
は取得できなかったため、内部でTCP APIを使って自前で実装した。

参考: 
- JAKA TCP Protocol-en-V2.0.6_20240128.pdf
- JAKA Python SDK User Manual V2.1.7.pdf
- nkawa/MQTT_Dobot_Nova2_Control/dobot_api.py
"""

from typing import Any, Dict, Optional, Tuple

import json
import logging
import socket
from time import sleep, perf_counter
import threading


logger = logging.getLogger(__name__)

class RCApi:
    def __init__(self, ip, port, timeout: Optional[float] = 60):
        self._ip = ip
        self._port = port
        self._socket = 0
        self._timeout = timeout
        self.__globalLock = threading.Lock()

    def _login(self):
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(self._timeout)
            self._socket.connect((self._ip, self._port))
        except Exception:
            logger.error("_login failed")

    def _send_data(self, string):
        try:
            self._socket.send(str.encode(string, 'utf-8'))
        except Exception:
            logger.error("_send_data failed")
            while True:
                try:
                    self._socket = self._reConnect(self._ip, self._port)
                    self._socket.send(str.encode(string, 'utf-8'))
                    break
                except Exception:
                    sleep(1)

    def _wait_reply(self):
        data = ""
        try:
            data = self._socket.recv(1024)
        except Exception as e:
            print(e)
            self._socket = self._reConnect(self._ip, self._port)

        finally:
            if len(data) == 0:
                data_str = data
            else:
                data_str = str(data, encoding="utf-8")
            return data_str

    def _close(self):
        if (self._socket != 0):
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
                self._socket.close()
            except socket.error as e:
                print(f"Error while closing socket: {e}")

    def _sendRecvMsg(self, string):
        with self.__globalLock:
            self._send_data(string)
            recvData = self._wait_reply()
            return recvData

    def __del__(self):
        self._close()

    def _reConnect(self, ip, port):
        while True:
            try:
                socket_ = socket.socket()
                socket_.settimeout(self._timeout)
                socket_.connect((ip, port))
                break
            except Exception:
                sleep(1)
        return socket_


class RC(RCApi):
    """制御用クライアント。"""
    def __init__(self, ip: str = "10.5.5.100", port: int = 10001) -> None:
        super().__init__(ip, port)
    
    def login(self):
        return self._login()

    def power_on(self) -> Tuple[int]:
        send = '{"cmdName": "power_on"}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def enable_robot(self) -> Tuple[int]:
        send = '{"cmdName": "enable_robot"}'
        recv = self._sendRecvMsg(send)
        return self._parse_results(recv)

    def disable_robot(self) -> Tuple[int]:
        send = '{"cmdName": "disable_robot"}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def clear_error(self):
        send = '{"cmdName":"clear_error"}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def collision_recover(self):
        return self.clear_error()

    def is_in_servomove(self) -> Tuple[int, int]:
        # 元のPython APIにはないが、TCP APIにある
        send = '{"cmdName":"is_in_servomove"}'
        recv = self._sendRecvMsg(send)
        ec = self._parse_error_code(recv)[0]
        ps = self._parse_results(recv)["in_servomove"]
        return (ec, ps)

    def servo_move_enable(self, enable: bool) -> Tuple[int]:
        rf = 1 if enable else 0
        send = f'{{"cmdName": "servo_move", "relFlag": {rf}}}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def servo_j(
        self,
        joint_pos: Tuple[float, float, float, float, float, float],
        move_mode: int,
    ) -> Tuple[int]:
        if move_mode not in [0, 1]:
            print("move_mode must be either of [0, 1]")
            return (-1,)
        cp = ",".join(map(str, joint_pos))
        sn = 1
        send = f'{{"cmdName": "servo_j", "jointPosition": [{cp}], "relFlag": {move_mode}, "stepNum": {sn}}}'
        recv = self._sendRecvMsg(send)
        return self._parse_results(recv)

    def protective_stop_status(self) -> Tuple[int, int]:
        # 元のPython APIにはないが、TCP APIにある
        send = f'{"cmdName": "protective_stop_status"}'
        recv = self._sendRecvMsg(send)
        ec = self._parse_results(recv)["errorCode"]
        ps = self._parse_results(recv)["protective_stop"]
        return (ec, ps)

    def servo_p(
        self,
        cartesian_pose: Tuple[float, float, float, float, float, float],
        move_mode: int,
    ) -> Tuple[int]:
        if move_mode not in [0, 1]:
            print("move_mode must be either of [0, 1]")
            return (-1,)
        cp = ",".join(map(str, cartesian_pose))
        sn = 1
        send = f'{{"cmdName": "servo_p", "catPosition": [{cp}], "relFlag": {move_mode}, "stepNum": {sn}}}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def get_joint_position(self):
        send = '{"cmdName":"get_joint_pos"}'
        recv = self._sendRecvMsg(send)
        ec = self._parse_error_code(recv)[0]
        # Assume "joint_pos": [j1,j2,j3,j4,j5,j6]
        query = '"joint_pos": ['
        i = recv.find(query)
        if i == -1:
            ec = -1
            return (ec,)
        j = i + len(query)
        k = recv[j:].find(']')
        if k == -1:
            ec = -1
            return (ec,)
        joint_pos = [float(x) for x in recv[j:][:k].split(",")]
        return (ec, joint_pos)

    def get_tcp_position(
        self
    ) -> Tuple[int, Tuple[float, float, float, float, float, float]]:
        send = '{"cmdName": "get_tcp_pos"}'
        recv = self._sendRecvMsg(send)
        ec = self._parse_error_code(recv)[0]
        # Assume "tcp_pos": [x,y,z,a,b,c]
        query = '"tcp_pos": ['
        i = recv.find(query)
        if i == -1:
            ec = -1
            return (ec,)
        j = i + len(query)
        k = recv[j:].find(']')
        if k == -1:
            ec = -1
            return (ec,)
        tcp_pos = [float(x) for x in recv[j:][:k].split(",")]
        return (ec, tcp_pos)

    def joint_move_extend(
        self,
        joint_pose: Tuple[float, float, float, float, float, float],
        move_mode: int,
        is_block: bool = True,
        speed: float = 1,
        accel: float = 1,
    ) -> Tuple[int]:
        if move_mode not in [0, 1]:
            print("move_mode must be either of [0, 1]")
            return (-1,)

        if not is_block:
            print("NotImplemented")
            return (-1,)
        
        if accel > 8000:
            print(f"accel > 8000 is not recommended: {accel}")
            return (-1,)

        ep = ",".join(map(str, joint_pose))
        send = f'{{"cmdName": "joint_move", "relFlag": {move_mode}, "jointPosition": [{ep}], "speed": {speed}, "accel": {accel}}}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def end_move(
        self,
        endPosition: Tuple[float, float, float, float, float, float],
        speed: float = 1,
        accel: float = 1
    ) -> Tuple[int]:
        ep = ",".join(map(str, endPosition))
        send = f'{{"cmdName": "end_move", "endPosition": [{ep}], "speed": {speed}, "accel": {accel}}}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def linear_move_extend(
        self,
        end_pos: Tuple[float, float, float, float, float, float],
        move_mode: int,
        is_block: bool = True,
        speed: float = 1,
        accel: float = 1,
        tol: float = 1,
    ) -> Tuple[int]:
        # TCP APIにはlinear_move_extendがなく、moveLで実装
        if move_mode not in [0, 1]:
            print("move_mode must be either of [0, 1]")
            return (-1,)

        if not is_block:
            print("NotImplemented")
            return (-1,)
        
        if accel > 8000:
            print(f"accel > 8000 is not recommended: {accel}")
            return (-1,)

        ep = ",".join(map(str, end_pos))
        send = f'{{"cmdName": "moveL", "relFlag": {move_mode}, "cartPosition": [{ep}], "speed": {speed}, "accel": {accel}, "tol": {tol}}}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def motion_abort(self) -> Tuple[int]:
        send = '{"cmdName": "stop_program"}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)
    
    def get_robot_state(self):
        send = '{"cmdName": "get_robot_state"}'
        recv = self._sendRecvMsg(send)
        return self._parse_results(recv)

    def logout(self) -> Tuple[int]:
        return self._logout_sdk()

    def _logout_sdk(self) -> None:
        send = '{"cmdName": "quit"}'
        recv = self._sendRecvMsg(send)
        return self._parse_error_code(recv)

    def _logout_socket(self) -> None:
        self._close()

    def _parse_results(self, valueRecv: str) -> Dict[str, Any]:
        # TCP API結果をパースする。エラーコード以外が必要な場合に使う。
        return json.loads(valueRecv)

    def _parse_error_code(self, valueRecv: str) -> Tuple[int]:
        # TCP API結果をパースする。エラーコードのみ必要な場合に使う。より高速。
        query = '"errorCode": "'
        i = valueRecv.find(query)
        if i == -1:
            return -1
        else:
            j = i + len(query)
            k = valueRecv[j:].find('"')
            return (int(valueRecv[j:][:k]),)

class RCFeedBack(RCApi):
    """状態取得用クライアント。"""
    def __init__(self, ip: str = "10.5.5.100", port: int = 10000) -> None:
        super().__init__(ip, port)
        self._callback = None

    def login(self):
        ret = self._login()
        self.__MyType = []
        self.__Lock = threading.Lock()
        feed_thread = threading.Thread(target=self.recvFeedData)
        feed_thread.daemon = True
        feed_thread.start()
        sleep(1)
        return ret

    def recvFeedData(self):
        """
        平均約30ms間隔で状態を取得する。
        """
        buffer = b''
        sep = b'{"len":'
        while True:
            unit_data = b''
            while True:
                # 単位データの先頭を探す
                if sep in buffer:
                    i = buffer.index(sep)
                    # buffer = sep + ...
                    buffer = buffer[i:]
                # 単位データの末端を探す
                # この時点でバッファ中に単位データが2つあるとき
                # 1つ目の単位データを取り出す
                # HACK: buffer[1:] = sep[1:] + ...に対しては
                # bufferの先頭のsepにはヒットしない
                if sep in buffer[1:]:
                    # HACK: 上のハックによるインデックスを元に戻す
                    i = buffer[1:].index(sep) + 1
                    unit_data = buffer[:i]
                    buffer = buffer[i:]
                # 単位データが取り出せれば抜ける
                if unit_data != b'':
                    break
                # なければデータをバッファに追加する
                data = self._socket.recv(4096)
                if not data:
                    self._socket = self._reConnect(self._ip, self._port)
                    buffer = b''
                    # 接続が解除されたらバッファはリセット
                    continue
                buffer += data            
            with self.__Lock:
                self.__MyType = json.loads(unit_data.decode())
                self.__MyType["timestamp"] = perf_counter()
                if self._callback is not None:
                    self._callback(self.__MyType)

    def feedBackData(self):
        with self.__Lock:
            return self.__MyType

    def set_callback(self, callback):
        self._callback = callback
