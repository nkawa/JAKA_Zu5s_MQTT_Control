from pyDHgripper import AG95


class AG95Control:
    # TODO: 要動作確認
    # NOTE: VRからどのような制御値が来るかわからないので現在は使用していない
    def __init__(self, **kwargs) -> None:
        self.port = kwargs.get("port", "/dev/ttyUSB0")
        self.hand = AG95(self.port)

    def connect_and_setup(self) -> bool:
        # ロボットと接続できていればTCPソケットのようなコネクションの作成は不要
        return True

    def disconnect(self) -> None:
        pass

    def grip(self) -> None:
        self.hand.set_pos(0)
    
    def release(self) -> None:
        self.hand.set_pos(1000)
