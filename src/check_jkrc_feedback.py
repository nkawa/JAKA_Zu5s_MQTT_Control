import time
from jaka_control.jkrc_feedback import RCFeedBack


if __name__ == '__main__':
    rc_feedback = RCFeedBack()
    def callback(data):
        print(f"Callback: {data}")
    rc_feedback.set_callback(callback)
    while True:
        data = rc_feedback.feedBackData()
        print(f"Feedback Data: {data}")
        time.sleep(1)
