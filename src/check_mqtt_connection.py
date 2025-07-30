from paho.mqtt import client as mqtt


MQTT_SERVER = "sora2.uclab.jp"


class MQTTClient:
    def __init__(self):
        pass

    def connect_mqtt(self):
        self.client = mqtt.Client()  
        # MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_forever(timeout=1)   # 通信処理開始

    def on_connect(self, client, userdata, flag, rc):
        print(f"Connected with result code {rc}")
    
    def on_disconnect(self, client, userdata, rc):
        print(f"Disconnected with result code {rc}")
    
    def on_message(self, client, userdata, msg):
        print(f"Message received. topic: {msg.topic} payload: {msg.payload.decode()}")
        # ここでメッセージの処理を行うことができます

if __name__ == "__main__":
    mqtt_client = MQTTClient()
    mqtt_client.connect_mqtt()
