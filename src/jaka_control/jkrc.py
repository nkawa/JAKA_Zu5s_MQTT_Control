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


class RC:
    """
    JAKAの制御用API。
    TCP/IPプロトコルを用いて実装。
    Python SDKのAPIを模倣しているが、実際にSDKとの比較はしていない。
    特にエラーコードが0でない場合の返り値は、SDKではエラーコードの可能性があるが、
    それでは不便なため、TCP/IPプロトコルの結果も返すようにしている。
    """
    def __init__(
        self,
        ip: str = "10.5.5.100",
        port: int = 10001,
        timeout: Optional[float] = 60,
    ) -> None:
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

    def login(self) -> Tuple[int] | Tuple[int, Any]:
        try:
            self._login()
        except Exception as e:
            return (-1, str(e))
        return (0,)

    def power_on(self) -> Tuple[int] | Tuple[int, Any]:
        send = '{"cmdName": "power_on"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def power_off(self) -> Tuple[int] | Tuple[int, Any]:
        send = '{"cmdName": "power_off"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def enable_robot(self) -> Tuple[int] | Tuple[int, Any]:
        send = '{"cmdName": "enable_robot"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def disable_robot(self) -> Tuple[int] | Tuple[int, Any]:
        send = '{"cmdName": "disable_robot"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def clear_error(self) -> Tuple[int] | Tuple[int, Any]:
        send = '{"cmdName":"clear_error"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def is_in_servomove(self) -> Tuple[int, bool] | Tuple[int, Any]:
        """Python SDKにはない"""
        send = '{"cmdName":"is_in_servomove"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        ps = ret["in_servomove"]
        return (ec, ps)

    def servo_move_enable(self, enable: bool) -> Tuple[int] | Tuple[int, Any]:
        rf = 1 if enable else 0
        send = f'{{"cmdName": "servo_move", "relFlag": {rf}}}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def servo_j(
        self,
        joint_pos: Tuple[float, float, float, float, float, float],
        move_mode: int,
        step_num: int = 1,
    ) -> Tuple[int] | Tuple[int, Any]:
        """
        joint_pos: unit is degree.
        move_move: 0 for absolute move, 1 for relative move.
        """
        assert move_mode in [0, 1]
        cp = ",".join(map(str, joint_pos))
        sn = step_num
        send = f'{{"cmdName": "servo_j", "jointPosition": [{cp}], "relFlag": {move_mode}, "stepNum": {sn}}}'
        recv = self._sendRecvMsg(send)
        ec = self._parse_error_code_fast(recv)
        if ec == 0:
            return (ec,)
        else:
            return (ec, recv)

    def get_joint_position(self) -> Tuple[
        int, Tuple[float, float, float, float, float, float]
    ] | Tuple[int, Any]:
        send = '{"cmdName":"get_joint_pos"}'
        recv = self._sendRecvMsg(send)
        ec = self._parse_error_code_fast(recv)
        if ec == 0:
            # Assume "joint_pos": [j1,j2,j3,j4,j5,j6]
            query = '"joint_pos": ['
            i = recv.find(query)
            if i != -1:
                j = i + len(query)
                k = recv[j:].find(']')
                if k != -1:
                    joint_pos = [float(x) for x in recv[j:][:k].split(",")]
                    return (ec, joint_pos)
        # ここまででパースの失敗またはエラーコードが0でない場合
        return (ec, recv)

    def get_tcp_position(self) -> Tuple[
        int, Tuple[float, float, float, float, float, float]
    ] | Tuple[int, Any]:
        send = '{"cmdName":"get_tcp_pos"}'
        recv = self._sendRecvMsg(send)
        ec = self._parse_error_code_fast(recv)
        if ec == 0:
            # Assume "tcp_pos": [x,y,z,a,b,c]
            query = '"tcp_pos": ['
            i = recv.find(query)
            if i != -1:
                j = i + len(query)
                k = recv[j:].find(']')
                if k != -1:
                    joint_pos = [float(x) for x in recv[j:][:k].split(",")]
                    return (ec, joint_pos)
        # ここまででパースの失敗またはエラーコードが0でない場合
        return (ec, recv)

    def joint_move_extend(
        self,
        joint_pose: Tuple[float, float, float, float, float, float],
        move_mode: int,
        is_block: bool = True,
        speed: float = 1,
        accel: float = 1,
    ) -> Tuple[int] | Tuple[int, Any]:
        assert move_mode in [0, 1]
        assert is_block
        assert accel <= 8000, f"accel = {accel} > 8000 is not recommended"

        ep = ",".join(map(str, joint_pose))
        send = f'{{"cmdName": "joint_move", "relFlag": {move_mode}, "jointPosition": [{ep}], "speed": {speed}, "accel": {accel}}}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def end_move(
        self,
        endPosition: Tuple[float, float, float, float, float, float],
        speed: float = 1,
        accel: float = 1
    ) -> Tuple[int] | Tuple[int, Any]:
        """Python SDKにはない"""
        ep = ",".join(map(str, endPosition))
        send = f'{{"cmdName": "end_move", "endPosition": [{ep}], "speed": {speed}, "accel": {accel}}}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def linear_move_extend(
        self,
        end_pos: Tuple[float, float, float, float, float, float],
        move_mode: int,
        is_block: bool = True,
        speed: float = 1,
        accel: float = 1,
        tol: float = 1,
    ) -> Tuple[int] | Tuple[int, Any]:
        # TODO: end_moveとどちらを使う

        # TCP APIにはlinear_move_extendがなく、moveLで実装
        assert move_mode in [0, 1]
        assert is_block
        assert accel <= 8000, f"accel = {accel} > 8000 is not recommended"

        ep = ",".join(map(str, end_pos))
        send = f'{{"cmdName": "moveL", "relFlag": {move_mode}, "cartPosition": [{ep}], "speed": {speed}, "accel": {accel}, "tol": {tol}}}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        return (ec,)

    def logout(self) -> Tuple[int] | Tuple[int, Any]:
        try:
            self._logout_socket()
            return (0,)
        except Exception as e:
            return (-1, str(e))

    def get_version(self) -> Tuple[int, str] | Tuple[int, Any]:
        """コントローラーのバージョンを取得する。Python SDKにはない"""       
        send = '{"cmdName": "get_version"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        version = ret["version"]
        return (ec, version)

    def emergency_stop_status(self) -> Tuple[int, int] | Tuple[int, Any]:
        """
        Whether the robot is in an emergency stop state.
        0 means not in an emergency stop state.
        1 means in an emergency stop state.
        """
        send = '{"cmdName": "emergency_stop_status"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        emergency_stop = ret["emergency_stop"]
        return (ec, emergency_stop)

    # def protective_stop_status(self) -> Tuple[int, int]:
    #     # 元のPython APIにはないが、TCP APIにある
    #     send = f'{"cmdName": "protective_stop_status"}'
    #     recv = self._sendRecvMsg(send)
    #     ec = self._parse_results(recv)["errorCode"]
    #     ps = self._parse_results(recv)["protective_stop"]
    #     return (ec, ps)

    # def servo_p(
    #     self,
    #     cartesian_pose: Tuple[float, float, float, float, float, float],
    #     move_mode: int,
    # ) -> Tuple[int]:
    #     if move_mode not in [0, 1]:
    #         print("move_mode must be either of [0, 1]")
    #         return (-1,)
    #     cp = ",".join(map(str, cartesian_pose))
    #     sn = 1
    #     send = f'{{"cmdName": "servo_p", "catPosition": [{cp}], "relFlag": {move_mode}, "stepNum": {sn}}}'
    #     recv = self._sendRecvMsg(send)
    #     return self._parse_error_code(recv)

    # def motion_abort(self) -> Tuple[int]:
    #     send = '{"cmdName": "stop_program"}'
    #     recv = self._sendRecvMsg(send)
    #     return self._parse_error_code(recv)
    
    def get_robot_state(self) -> Tuple[int, int, int] | Tuple[int, Any]:
        send = '{"cmdName": "get_robot_state"}'
        ec, ret, recv = self._process(send)
        if ec != 0:
            return (ec, recv)
        enabled = ret["enable"] == "robot_enabled"
        powered_on = ret["power"] == "powered_on"
        return (ec, int(powered_on), int(enabled))

    # def _logout_sdk(self) -> None:
    #     # NOTE: "Quit connection"とのことだが具体的にどうなっているかは不明
    #     send = '{"cmdName": "quit"}'
    #     recv = self._sendRecvMsg(send)
    #     return self._parse_error_code(recv)

    def _logout_socket(self) -> None:
        self._close()

    def _parse_results(self, valueRecv: str) -> Dict[str, Any]:
        # TCP API結果をパースする。エラーコード以外が必要な場合に使う。
        return json.loads(valueRecv)
    
    def _parse_error_code_fast(self, valueRecv: str) -> int:
        # TCP API結果をパースする。エラーコードのみ必要な場合に使う。より高速。
        query = '"errorCode": "'
        i = valueRecv.find(query)
        if i == -1:
            return -1
        else:
            j = i + len(query)
            k = valueRecv[j:].find('"')
            error_code = valueRecv[j:][:k]
            return int(error_code)

    def _process(self, send: str) -> Tuple[int, Any, str]:
        recv = self._sendRecvMsg(send)
        ret = self._parse_results(recv)
        ec = int(ret["errorCode"])
        return (ec, ret, recv)
