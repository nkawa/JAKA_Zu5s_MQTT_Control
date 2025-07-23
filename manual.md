# JAKA_Zu5s_MQTT_Control マニュアル

NOTE(20250723): 作成中

## クイックスタート

** 物理的な接続設定 **

- コントロールキャビネット（銀の箱）の電源ケーブルをコンセントに挿す
- JAKA Zu 5s ロボットと制御プログラムを実行するPCをLANケーブルで接続する
- DH Robotics AG95 ハンドとPCをUSBケーブルで接続する

** Jaka Zu 5s ロボットへの接続設定例 **

```
# PCのアドレス
sudo ip addr add 10.5.5.1/24 dev <device-name>
sudo ip link set <device-name> up
# ロボットのアドレス
ping 10.5.5.100
```

** DH Robotics AG95 ハンドへの接続設定例 **

```sh
sudo chmod 777 /dev/ttyUSB0
sudo usermod -aG dialout <user-name>
```

** 必要なライブラリのインストール **

```sh
pip install -r src/requirements.txt
```

** プログラムの実行 **

```sh
python src/main.py
```
