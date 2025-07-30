import datetime
import logging

from jaka_control.jaka_robot import JakaRobot, JakaRobotFeedback


class MicrosecondFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        dt = datetime.datetime.fromtimestamp(record.created)
        if datefmt:
            s = dt.strftime(datefmt)
            # %f をマイクロ秒で置換
            s = s.replace('%f', f"{dt.microsecond:06d}")
            return s
        else:
            return super().formatTime(record, datefmt)

def make_logger(name):
    logger = logging.getLogger(name)
    handler = logging.StreamHandler()
    formatter = MicrosecondFormatter(
        "[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S.%f",
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    return logger


if __name__ == '__main__':
    check_move = False
    ctrl_logger = make_logger("CTRL-ROBOT")
    mon_logger = make_logger("MON-ROBOT")
    robot = JakaRobot(
        logger=ctrl_logger,
    )
    robot_feedback = JakaRobotFeedback(
        logger=mon_logger,
        save_feed=True,
    )
    try:
        robot.start()
        robot_feedback.start()

        print(f"{robot.emergency_stop_status()=}")
        print(f"{robot.is_powered_on()=}")

        robot.enable()
        print(f"{robot.is_enabled()=}")

        robot.clear_error()

        print(f"{robot.get_current_joint()=}")
        print(f"{robot.get_current_pose()=}")

        joint = robot.get_current_joint()
        if check_move:
            robot.move_joint(joint)
            robot.move_joint_until_completion(joint)

        pose = robot.get_current_pose()
        if check_move:
            robot.move_pose(pose)
            robot.move_pose_until_completion(pose)
        
        print(f"{robot.is_in_servomove()=}")
        try:
            if check_move:
                robot.move_joint_servo([0, 0, 0, 0, 0, 0])
        except Exception as e:
            ctrl_logger.exception("Exception during servo move")
            print(f"{robot.is_powered_on()=}")
            print(f"{robot.is_enabled()=}")
            print(f"{robot_feedback.get_cur_error_info_all()=}")
            robot.clear_error()
            print(f"{robot_feedback.get_cur_error_info_all()=}")
        
        # TODO: エラーが発生した場合に電源、イネーブル状態がどうなっているか確認すること
        # 多分解除されていないと思われる

        robot.enter_servo_mode()
        print(f"{robot.is_in_servomove()=}")
        try:
            if check_move:
                robot.move_joint_servo([0, 0, 0, 0, 0, 0])
        except Exception as e:
            ctrl_logger.exception("Exception during servo move")
            print(f"{robot.is_powered_on()=}")
            print(f"{robot.is_enabled()=}")
            robot.clear_error()
        robot.leave_servo_mode()

    except Exception as e:
        ctrl_logger.exception("Exception")
        del robot
