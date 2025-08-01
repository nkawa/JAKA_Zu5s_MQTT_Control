import datetime
import logging
import time

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
    """
    # check_move = False
    # robot.enable mistake
    [2025-08-01 11:21:44.048431][CTRL-ROBOT][INFO] start
    [2025-08-01 11:21:44.049746][MON-ROBOT][INFO] start
    robot.emergency_stop_status()=True
    robot.is_powered_on()=True
    [2025-08-01 11:21:45.056377][CTRL-ROBOT][INFO] enable
    [2025-08-01 11:21:45.057652][CTRL-ROBOT][ERROR] Exception
    Traceback (most recent call last):
    File "/home/user/JAKA_Zu5s_MQTT_Control/src/check_jaka_robot.py", line 49, in <module>
        robot.enable()
    File "/home/user/JAKA_Zu5s_MQTT_Control/src/jaka_control/jaka_robot.py", line 66, in enable
        if not self.is_enabled():
    File "/home/user/JAKA_Zu5s_MQTT_Control/src/jaka_control/jaka_robot.py", line 213, in is_enabled
        raise JakaRobotError(res[1])
    jaka_control.jaka_robot.JakaRobotError: 1
    [2025-08-01 11:21:45.059513][CTRL-ROBOT][INFO] leave_servo_mode
    [2025-08-01 11:21:45.375463][MON-ROBOT][ERROR] Error in feedback: {'errcode': '0x10f0005', 'errmsg': 'robot servo disabled', 'powered_on': 1, 'enabled': True, 'paused': False, 'on_soft_limit': 0, 'emergency_stop': 0, 'protective_stop': 0}
    [2025-08-01 11:21:45.632082][CTRL-ROBOT][INFO] Robot deleted

    # check_move = True
    [2025-08-01 11:26:38.117615][CTRL-ROBOT][INFO] start
    [2025-08-01 11:26:38.118869][MON-ROBOT][INFO] start
    robot.emergency_stop_status()=True
    robot.is_powered_on()=True
    [2025-08-01 11:26:39.121914][CTRL-ROBOT][INFO] enable
    robot.is_enabled()=True
    robot.get_current_joint()=[-261.539492, 107.410084, 76.54078, 89.997822, -93.317526, -1.545968]
    robot.get_current_pose()=[-29.877919, -613.279714, 390.613715, 175.956975, -3.201816, -169.766085]
    robot.is_in_servomove()=False
    [2025-08-01 11:26:41.274632][CTRL-ROBOT][ERROR] Exception during servo move
    Traceback (most recent call last):
    File "/home/user/JAKA_Zu5s_MQTT_Control/src/check_jaka_robot.py", line 91, in <module>
        robot.move_joint_servo([0, 0, 0, 0, 0, 0])
    File "/home/user/JAKA_Zu5s_MQTT_Control/src/jaka_control/jaka_robot.py", line 174, in move_joint_servo
        raise JakaRobotError(res[1])
    jaka_control.jaka_robot.JakaRobotError: {"errorCode": "-1", "errorMsg": "servoj command can only be excuted in servo move mode", "cmdName": "servo_j"}
    robot.is_powered_on()=True
    robot.is_enabled()=True
    # No error detected in feedback, even if waited 1 second. Actually, save_feed shows not error in feedback
    # Error above (slave mode error) may be error in control aspect, but not in feedback aspect
    # This means you cannot trust feedback error to recover automatically
    robot_feedback.get_cur_error_info_all()=[]
    robot_feedback.get_cur_error_info_all()=[]
    [2025-08-01 11:26:41.286827][CTRL-ROBOT][INFO] enter_servo_mode
    robot.is_in_servomove()=True
    [2025-08-01 11:26:41.295971][CTRL-ROBOT][INFO] leave_servo_mode

    """
    check_move = True
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
            time.sleep(1)
            print(f"{robot_feedback.get_cur_error_info_all()=}")
            robot.clear_error()
            print(f"{robot_feedback.get_cur_error_info_all()=}")

        robot.enter_servo_mode()
        print(f"{robot.is_in_servomove()=}")
        try:
            if check_move:
                robot.move_joint_servo([0, 0, 0, 0, 0, 0])
        except Exception as e:
            ctrl_logger.exception("Exception during servo move")
            print(f"{robot.is_powered_on()=}")
            print(f"{robot.is_enabled()=}")
            time.sleep(1)
            print(f"{robot_feedback.get_cur_error_info_all()=}")
            robot.clear_error()
            print(f"{robot_feedback.get_cur_error_info_all()=}")
        robot.leave_servo_mode()

    except Exception as e:
        ctrl_logger.exception("Exception")
        del robot
