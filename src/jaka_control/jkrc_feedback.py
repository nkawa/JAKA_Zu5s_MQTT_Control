from typing import Any, Dict, Optional, Tuple

import json
import logging
import socket
from time import sleep, perf_counter
import threading


logger = logging.getLogger(__name__)


class RCFeedBack:
    """状態取得用クライアント。"""
    def __init__(
        self,
        ip: str = "10.5.5.100",
        port: int = 10000,
        timeout: Optional[float] = 60,
    ) -> None:
        self._ip = ip
        self._port = port
        self._socket = None
        self._timeout = timeout
        self._callback = None

    def _close(self):
       if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except Exception:
                logger.exception("Error while socket shutdown")
            try:
                self._socket.close()
            except Exception:
                logger.exception("Error while socket close")
            self._socket = None

    def __del__(self):
        self._close()

    def _reConnect(self):
        while True:
            try:
                self._close()
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket.settimeout(self._timeout)
                self._socket.connect((self._ip, self._port))
            except Exception:
                self._close()
                logger.exception("Error while reconnecting")
                sleep(1)

    def login(self):
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(self._timeout)
            self._socket.connect((self._ip, self._port))
        except Exception as e:
            if self._socket is not None:
                self._socket.close()
                self._socket = None
                raise e
        self.__MyType = {}
        self.__Lock = threading.Lock()
        feed_thread = threading.Thread(target=self.recvFeedData)
        feed_thread.daemon = True
        feed_thread.start()
        sleep(1)

    def recvFeedData(self):
        """
        デフォルトの設定で、実測した限りでは、平均約30ms間隔で状態を取得する。
        出力するデータの種類を減らす設定が該当バージョンで可能ならば、
        高速化が期待できる。
        """
        # 受信データは、'<unit_data><unit_data>...'
        # (具体的には'{"len": ...}{"len": ...}...')の形式で送られてくるが、
        # 1回のソケット受信では途中の切り取りを受信するため、
        # バッファを用意して、単位データごとに切り出して処理する。
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
                    self._reConnect()
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
