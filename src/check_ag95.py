import time
from pyDHgripper import AG95


if __name__ == '__main__':
    """
    # Reference:
    # AG-series-Short-ManualModbus-RTUv2.3_DH-Robotics.pdf
    # prints are shown in stdout at once at last, why?

    # set_pos seems not blocking
    # set_pos is done correctly

    gripper.read_state()=1
    gripper.read_pos()=1000
    gripper.set_pos(0)=None
    gripper.read_pos()=994
    gripper.set_pos(1000)=None
    gripper.read_pos()=977
    """
    check_move = True
    # この呼び出しでグリッパーの開閉が1度実行される
    gripper = AG95(port='/dev/ttyUSB0')
    # 0：In motion;
    # 1：Reached position
    # 2：Object caught
    # 3：Object dropped
    print(f"{gripper.read_state()=}")
    print(f"{gripper.read_pos()=}")

    # Grip
    if check_move:
        print(f"{gripper.set_pos(0)=}")
        # 全く待機しないと閉じないが、待機時間は少しで良い
        time.sleep(1)
    print(f"{gripper.read_pos()=}")
    # Release
    if check_move:
        print(f"{gripper.set_pos(1000)=}")
        time.sleep(1)
    print(f"{gripper.read_pos()=}")

    # Velocity low
    # 速度を設定するだけでは何も起こらない
    # 遅い速度を設定しても、グリッパーの動作が変わらないように見える
    # ドキュメントと動作確認の結果、力のみ速度に影響する
    print(f"{gripper.set_vel(1)=}")
    time.sleep(1)
    # Grip
    if check_move:
        print(f"{gripper.set_pos(0)=}")
        time.sleep(1)
    print(f"{gripper.read_pos()=}")
    # Release
    if check_move:
        print(f"{gripper.set_pos(1000)=}")
        time.sleep(1)

    # Velocity high
    print(f"{gripper.set_vel(500)=}")
    time.sleep(1)
    # Grip
    if check_move:
        print(f"{gripper.set_pos(0)=}")
        time.sleep(1)
    print(f"{gripper.read_pos()=}")
    # Release
    if check_move:
        print(f"{gripper.set_pos(1000)=}")
        time.sleep(1)

    # Force low
    print(f"{gripper.set_force(20)=}")
    time.sleep(1)
    # Grip
    if check_move:
        print(f"{gripper.set_pos(0)=}")
        time.sleep(1)
    print(f"{gripper.read_pos()=}")
    # Release
    if check_move:
        print(f"{gripper.set_pos(1000)=}")
        time.sleep(1)

    print(f"{gripper.set_force(100)=}")
    # Grip
    if check_move:
        print(f"{gripper.set_pos(0)=}")
        time.sleep(1)
    print(f"{gripper.read_pos()=}")
    # Release
    if check_move:
        print(f"{gripper.set_pos(1000)=}")
        time.sleep(1)
