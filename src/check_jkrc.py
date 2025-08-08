from jaka_control.jkrc import RC


class JakaRobotError(Exception):
    pass


"""
# check_move = False
rc.login()=(0,)
rc.get_robot_state()=(0, 0, 0)
rc.get_version()=(0, '1.5.14_13_X64')
rc.emergency_stop_status()=(1, '{"errorCode": "1", "errorMsg": "Exception:function call failed"}')
rc.power_on()=(0,)
rc.get_robot_state()=(0, 1, 0)
rc.enable_robot()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.clear_error()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.get_joint_position()=(0, [-263.405107, 106.408955, 75.061059, 89.814487, -93.426702, -13.407083])
rc.get_tcp_position()=(0, [-50.169481, -610.80576, 413.795359, 177.953437, -3.034099, -159.905394])
rc.is_in_servomove()=(0, False)
rc.servo_move_enable(True)=(0,)
rc.is_in_servomove()=(0, True)
rc.servo_move_enable(False)=(0,)
rc.disable_robot()=(0,)
rc.power_off()=(0,)

# check_move = True
rc.login()=(0,)
rc.get_robot_state()=(0, 0, 0)
rc.get_version()=(0, '1.5.14_13_X64')
rc.emergency_stop_status()=(1, '{"errorCode": "1", "errorMsg": "Exception:function call failed"}')
rc.power_on()=(0,)
rc.get_robot_state()=(0, 1, 0)
rc.enable_robot()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.clear_error()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.get_joint_position()=(0, [-263.403734, 106.909519, 75.54789, 89.850193, -93.426702, -13.407083])
rc.get_tcp_position()=(0, [-49.993733, -612.200172, 404.404579, 176.95742, -2.79482, -159.853377])
rc.joint_move_with_acc(joint, 0, speed=10, accel=10)=(2, '{"errorCode": "2", "errorMsg": "call joint_move failed", "cmdName": "joint_move"}')
rc.joint_move_with_acc([0, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=(0,)
rc.end_move(pose, speed=10, accel=10)=(2, '{"errorCode": "2", "errorMsg": "call end_move failed", "cmdName": "end_move"}')
rc.linear_move_extend(pose, 0, speed=10, accel=10)=(2, '{"errorCode": "2", "errorMsg": "call moveL failed", "cmdName": "moveL"}')
rc.linear_move_extend([0, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=(0,)
rc.is_in_servomove()=(0, False)
rc.servo_j([0, 0, 0, 0, 0, 0], 1)=(-1, '{"errorCode": "-1", "errorMsg": "servoj command can only be excuted in servo move mode", "cmdName": "servo_j"}')
rc.servo_move_enable(True)=(0,)
rc.is_in_servomove()=(0, True)
rc.servo_j([0, 0, 0, 0, 0, 0], 1)=(0,)
rc.servo_move_enable(False)=(0,)
rc.disable_robot()=(0,)
rc.power_off()=(0,)

# check_move = True
# 2nd
rc.login()=(0,)
rc.get_robot_state()=(0, 0, 0)
rc.get_version()=(0, '1.5.14_13_X64')
rc.emergency_stop_status()=(1, '{"errorCode": "1", "errorMsg": "Exception:function call failed"}')
rc.power_on()=(0,)
rc.get_robot_state()=(0, 1, 0)
rc.enable_robot()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.clear_error()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.get_joint_position()=(0, [-263.403734, 107.40871, 76.043649, 89.887272, -93.426016, -13.407083])
rc.get_tcp_position()=(0, [-49.852504, -613.410281, 394.960563, 175.953287, -2.551933, -159.806574])
rc.joint_move_with_acc([0, 0, 0, 0, 0, 10], 1, speed=10, accel=10)=(0,)
# joint was wrong here
rc.joint_move_with_acc(joint, 0, speed=10, accel=10)=(2, '{"errorCode": "2", "errorMsg": "call joint_move failed", "cmdName": "joint_move"}')
# but succeeding command is done without clear_error
rc.linear_move_extend([10, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=(0,)
rc.end_move(pose, speed=10, accel=10)=(2, '{"errorCode": "2", "errorMsg": "call end_move failed", "cmdName": "end_move"}')
rc.linear_move_extend([10, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=(0,)
rc.linear_move_extend(pose, 0, speed=10, accel=10)=(2, '{"errorCode": "2", "errorMsg": "call moveL failed", "cmdName": "moveL"}')
rc.is_in_servomove()=(0, False)
rc.servo_j([0, 0, 0, 0, 0, 0], 1)=(-1, '{"errorCode": "-1", "errorMsg": "servoj command can only be excuted in servo move mode", "cmdName": "servo_j"}')
rc.servo_move_enable(True)=(0,)
rc.is_in_servomove()=(0, False)
rc.servo_j([0, 0, 0, 0, 0, 0], 1)=(-1, '{"errorCode": "-1", "errorMsg": "servoj command can only be excuted in servo move mode", "cmdName": "servo_j"}')
rc.servo_move_enable(False)=(0,)
rc.disable_robot()=(0,)
rc.power_off()=(0,)

# check_move = True
# 3rd
rc.login()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.get_version()=(0, '1.5.14_13_X64')
rc.emergency_stop_status()=(1, '{"errorCode": "1", "errorMsg": "Exception:function call failed"}')
rc.power_on()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.enable_robot()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.clear_error()=(0,)
rc.get_robot_state()=(0, 1, 1)
rc.get_joint_position()=(0, [-261.539269, 107.261397, 76.208493, 89.979374, -93.315612, -1.542663])
rc.get_tcp_position()=(0, [-29.852503, -613.410289, 394.960562, 176.457188, -3.215444, -169.797277])
rc.joint_move_with_acc([0, 0, 0, 0, 0, 10], 1, speed=10, accel=10)=(0,)
rc.joint_move_with_acc(joint, 0, speed=10, accel=10)=(0,)
rc.linear_move_extend([10, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=(0,)
rc.end_move(pose, speed=10, accel=10)=(0,)
rc.linear_move_extend([10, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=(0,)
rc.linear_move_extend(pose, 0, speed=10, accel=10)=(0,)
rc.is_in_servomove()=(0, False)
rc.servo_j([0, 0, 0, 0, 0, 0], 1)=(-1, '{"errorCode": "-1", "errorMsg": "servoj command can only be excuted in servo move mode", "cmdName": "servo_j"}')
rc.servo_move_enable(True)=(0,)
rc.is_in_servomove()=(0, False)
rc.servo_j([0, 0, 0, 0, 0, 0], 1)=(-1, '{"errorCode": "-1", "errorMsg": "servoj command can only be excuted in servo move mode", "cmdName": "servo_j"}')
rc.servo_move_enable(False)=(0,)
rc.disable_robot()=(0,)
rc.power_off()=(0,)
"""

if __name__ == '__main__':
    check_move = True
    rc = RC()
    try:
        print(f"{rc.login()=}")
        print(f"{rc.get_robot_state()=}")

        print(f"{rc.get_version()=}")
        print(f"{rc.emergency_stop_status()=}")

        print(f"{rc.power_on()=}")
        print(f"{rc.get_robot_state()=}")

        print(f"{rc.enable_robot()=}")
        print(f"{rc.get_robot_state()=}")

        print(f"{rc.clear_error()=}")
        print(f"{rc.get_robot_state()=}")

        print(f"{rc.get_joint_position()=}")
        print(f"{rc.get_tcp_position()=}")

        res = rc.get_joint_position()
        if res[0] != 0:
            raise JakaRobotError(res[1])
        joint = res[1]

        if check_move:
            print(f"{rc.joint_move_with_acc([0, 0, 0, 0, 0, 10], 1, speed=10, accel=10)=}")
            print(f"{rc.joint_move_with_acc(joint, 0, speed=10, accel=10)=}")

        res = rc.get_tcp_position()
        if res[0] != 0:
            raise JakaRobotError(res[1])
        pose = res[1]

        if check_move:
            print(f"{rc.linear_move_extend([10, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=}")
            print(f"{rc.end_move(pose, speed=10, accel=10)=}")
            print(f"{rc.linear_move_extend([10, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=}")
            print(f"{rc.linear_move_extend(pose, 0, speed=10, accel=10)=}")

        print(f"{rc.is_in_servomove()=}")
        try:
            if check_move:
                print(f"{rc.servo_j([0, 0, 0, 0, 0, 0], 1)=}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"{rc.get_robot_state()=}")
            print(f"{rc.clear_error()=}")
            print(f"{rc.get_robot_state()=}")

        print(f"{rc.servo_move_enable(True)=}")
        print(f"{rc.is_in_servomove()=}")
        try:
            if check_move:
                print(f"{rc.servo_j([0, 0, 0, 0, 0, 0], 1)=}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"{rc.get_robot_state()=}")
        print(f"{rc.servo_move_enable(False)=}")

    finally:
        print(f"{rc.disable_robot()=}")
        print(f"{rc.power_off()=}")
