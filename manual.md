# JAKA_Zu5s_MQTT_Control マニュアル

## クイックスタート：MetaworkMQTTでのロボット制御

**1. ロボットの起動**

- Jaka Zu 5s のコントロールキャビネット（銀の箱）の電源ケーブルをコンセントに挿す
- DH Robotics AG95 ハンドの電源ケーブルをコンセントに挿す
- 緊急停止ボタンがOFFである（上に引いてある）ことを確認する。また、ロボット制御時に、緊急停止したい場合は緊急停止ボタンをONにする（押す）ことを確認しておく

**2. ロボットとPCの接続**

- ロボットと制御プログラムを実行するPCをLANケーブルで接続する
- ハンドとPCをUSBケーブルで接続する
- 以下の接続設定を行う

ロボットとの通信:

```sh
# LANデバイスにアドレスを設定 (一時的、再起動すると消える)
sudo ip addr add 10.5.5.1/24 dev <device-name>
# LANデバイスを有効にする
sudo ip link set <device-name> up
# LANデバイスの状態、通信可能か (state UP)、アドレスが設定されているか確認する
ip addr show <device-name>
# ロボットへの導通確認
ping 10.5.5.100
```

ハンドとの通信 (一度設定すれば再起動しても設定は残る):

```sh
sudo chmod 777 /dev/ttyUSB0
sudo usermod -aG dialout <user-name>
```

**3. プログラムの設定**

必要なライブラリのインストール：必要な場合のみ行う。

```sh
pip install -r src/requirements.txt
```

コマンド`python`のシンボリックリンクをたどった最終的なバイナリに対して、リアルタイムスケジューラを利用するための権限を付与する。例えば、バイナリが`/usr/bin/python3.10`の場合は、

```sh
sudo setcap cap_sys_nice=eip /usr/bin/python3.10
```

**4. プログラムの実行**

MQTTサーバーが`sora2.uclab.jp`で動いていることが前提となる。Linuxでは、以下のコマンドでロボット制御コードを実行する。

```sh
python src/main.py
```

GUI画面が起動する。ロボット制御用のボタン、状態表示のランプ、状態表示のログ画面が存在する。

**NOTE(2025/07/30): 現時点ではツールチェンジ・ボタンに対応していないので、`ToolChange`ボタンは押下できないようになっている。その他にも対応していない機能・ボタンが存在する。**

ロボット制御側で、MQTTでの制御を受け付けるようにするためには、以下の手順で操作を行う。

1. `ConnectRobot`でロボットに接続
2. `ConnectMQTT`でMQTTサーバーに接続
3. `EnableRobot`でロボットのモーターをONにする。ロボットのモーターがONになっていれば、`Enabled`のランプが緑色に点灯する
4. `StartMQTTControl`でMQTTでの制御を受け付けるようにする。成功すれば`MQTTControl`のランプが緑色に点灯する

この状態で、VRゴーグルから`https://sora2.uclab.jp/<name>` (TODO: nameにはJAKA用のアドレスが入る)にアクセスし、タブの`Enter VR`または画面右下の`AR`を押すことで、VRコントローラでロボットを制御できるようになる。

ロボット制御側で、MQTTでの制御を止める場合は、`StopMQTTControl`を押す。成功すれば`MQTTControl`のランプが消える。

MQTTでの制御中に、エラーが起きた場合は、自動復帰可能なエラー（例：速度・加速度エラー）であれば、1度自動復帰を試みる。成功した場合、そのままMQTTでの制御が可能である。失敗した場合は、MQTTでの制御が止まる。また、自動復帰不可能なエラーの場合も、MQTTでの制御が止まる。

エラーが残っている場合、`Error`のランプが赤色に点灯する。エラーが起こると、モーターもOFFになることが多い。モーターを再度ONにする場合は、`ClearError`でエラー表示を消した後、`EnableRobot`を押せばよい。

`ReleaseHand`でハンドを最大まで開くことができる。`TidyPose`でロボットの先端がロボットの台の中央付近になり、先端が縦向きになる、片付け用の姿勢に移動できる。`DefaultPose`はロボットの先端がロボットの台の中央付近になるが、先端が横向きになる、姿勢に移動できる。`DisableRobot`でモーターをOFFにできる。閉じるボタンでロボット制御コードを終了できる。

**現在の位置から`TidyPose`で移動する場所への移動の途中で障害物にぶつかることがないように、現在の位置を事前に、ワークや障害物から離れた位置で、作業台から上に十分離れた位置になるようにしておくこと。**

**NOTE(20250808): 現状はVRの姿勢と実機の姿勢が必ず一致している状態から`StartMQTTControl`を始めること。そのために、`StartMQTTControl`の前には毎回必ず`EnableRobot`を行うこと。**

**NOTE(20250822): 現在ToolChange、DemoPutDownBox、ReleaseHandは使用しないためボタンは隠している。**

ログ出力として、イベントログ（MQTT受信、制御、モニタ、GUIプロセスの重要なイベント・エラーのログ）は、GUI、標準出力、ファイルに同じ内容を出力している。イベントログファイルは、ディレクトリ`log/<日付>/<時刻>`（GUI起動時の日時）の`log.txt`として保存される。デフォルトでは、ロボットへの制御値とロボットの状態値も、それぞれ`control.jsonl`、`state.jsonl`として同ディレクトリに保存される。制御値と状態値は、MQTT制御時のみ保存される。`ChangeLogFile`ボタンを押すと、その日時のディレクトリにログの出力先が切り替わる。この機能は、MQTT制御時でもそうでないときでも使用可能である。ロボットへの制御値と状態値は、環境変数でSAVE='false'と指定すれば保存されなくなる。

環境変数:

`src/jaka_control/.env`に環境変数を配置することでプログラムの挙動を変更できる:

```sh
MQTT_MODE='metawork'
MQTT_SERVER='sora2.uclab.jp'
ROBOT_UUID='JAKA-control'  # 現状はROBOT_MODELと一致させているが別にそうでなくても良い
ROBOT_MODEL='JAKA-control'  # VR側のコードと対応させること
MQTT_MANAGE_TOPIC='mgr'
MQTT_CTRL_TOPIC='control'
MQTT_ROBOT_STATE_TOPIC='robot'
MQTT_FORMAT='Jaka-Control-IK'
ROBOT_IP='10.5.5.100'
SAVE='false'
MOVE='true'  # `false`でロボットに接続するが制御値は送信しない
MOCK='false'  # `true`でロボットに接続せずテストモックを用いる
```
