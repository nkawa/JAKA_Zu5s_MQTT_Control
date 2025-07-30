from jaka_control.jkrc import RC


if __name__ == '__main__':
    check_move = False
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

        joint = rc.get_joint_position()
        if check_move:
            print(f"{rc.joint_move_with_acc(joint, 0, speed=10, accel=10)=}")
            print(f"{rc.joint_move_with_acc([0, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=}")

        pose = rc.get_tcp_position()
        if check_move:
            print(f"{rc.end_move(pose, speed=10, accel=10)=}")
            print(f"{rc.linear_move_extend(pose, 0, speed=10, accel=10)=}")
            print(f"{rc.linear_move_extend([0, 0, 0, 0, 0, 0], 1, speed=10, accel=10)=}")

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
