import datetime
import logging
import json
import logging.handlers
import time
import tkinter as tk
import multiprocessing
from tkinter import scrolledtext
from typing import Optional

from jaka_control.jaka_zu_mqtt_control import ProcessManager
from jaka_control.tools import tool_infos


tool_ids = [tool_info["id"] for tool_info in tool_infos]


class ToolChangePopup(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.result = None
        # 親ウインドウの手前に表示
        self.transient(parent)
        # すべてのイベントをポップアップで捕捉
        self.grab_set()
        self.title("Tool Change")
        # ポップアップを閉じるときの処理を上書き
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        
        tk.Label(self, text="Select a tool:").pack(pady=10)
        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=5)
        names = [f"Tool {i}" for i in tool_ids] + ["Cancel"]
        for name in names:
            b = tk.Button(btn_frame, text=name, width=10,
                          command=lambda n=name: self.button_pressed(n))
            b.pack(side=tk.LEFT, padx=5)

        # ポップアップが閉じるまで待機    
        self.wait_window()
    
    def button_pressed(self, name):
        if name == "Cancel":
            self.result = None
        else:
            self.result = int(name.split(" ")[1])
        self.destroy()

    def on_close(self):
        self.result = None
        self.destroy()


class GUILoggingHandler(logging.Handler):
    def __init__(self, gui: "MQTTWin") -> None:
        super().__init__()
        self.gui = gui
        self._closed = False 

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            # GUIはメインスレッドで更新する
            self.gui.root.after(0, self.gui.update_log, msg)
        except Exception:
            self.handleError(record)

    def close(self) -> None:
        # 並列処理時のログハンドラの多重closeを防ぐ
        if not self._closed:
            super().close()
            self._closed = True

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


class QueueListener(object):
    """
    NOTE: Copied from logging.handlers.QueueListener.

    This class implements an internal threaded listener which watches for
    LogRecords being added to a queue, removes them and passes them to a
    list of handlers for processing.
    """
    _sentinel = None

    def __init__(self, queue, *handlers, respect_handler_level=False):
        """
        Initialise an instance with the specified queue and
        handlers.
        """
        self.queue = queue
        self.handlers = handlers
        self._thread = None
        self.respect_handler_level = respect_handler_level

    def dequeue(self, block):
        """
        Dequeue a record and return it, optionally blocking.

        The base implementation uses get. You may want to override this method
        if you want to use timeouts or work with custom queue implementations.
        """
        return self.queue.get(block)

    def start(self):
        """
        Start the listener.

        This starts up a background thread to monitor the queue for
        LogRecords to process.
        """
        self._thread = t = threading.Thread(target=self._monitor)
        t.daemon = True
        t.start()

    def prepare(self, record):
        """
        Prepare a record for handling.

        This method just returns the passed-in record. You may want to
        override this method if you need to do any custom marshalling or
        manipulation of the record before passing it to the handlers.
        """
        return record

    def handle(self, record):
        """
        Handle a record.

        This just loops through the handlers offering them the record
        to handle.
        """
        record = self.prepare(record)
        for handler in self.handlers:
            if not self.respect_handler_level:
                process = True
            else:
                process = record.levelno >= handler.level
            if process:
                handler.handle(record)

    def _monitor(self):
        """
        Monitor the queue for records, and ask the handler
        to deal with them.

        This method runs on a separate, internal thread.
        The thread will terminate if it sees a sentinel object in the queue.
        """
        q = self.queue
        has_task_done = hasattr(q, 'task_done')
        while True:
            try:
                record = self.dequeue(True)
                if record is self._sentinel:
                    if has_task_done:
                        q.task_done()
                    break
                self.handle(record)
                if has_task_done:
                    q.task_done()
            except queue.Empty:
                break

    def enqueue_sentinel(self):
        """
        This is used to enqueue the sentinel record.

        The base implementation uses put_nowait. You may want to override this
        method if you want to use timeouts or work with custom queue
        implementations.
        """
        self.queue.put_nowait(self._sentinel)

    def stop(self):
        """
        Stop the listener.

        This asks the thread to terminate, and then waits for it to do so.
        Note that if you don't call this before your application exits, there
        may be some records still left on the queue, which won't be processed.
        """
        self.enqueue_sentinel()
        self._thread.join()
        self._thread = None


class MQTTWin:
    def __init__(self, root, use_joint_monitor_plot: bool = False):
        self.use_joint_monitor_plot = use_joint_monitor_plot
        self.pm = ProcessManager()
        log_queue = self.pm.log_queue
        self.setup_logging(log_queue=log_queue)
        self.setup_logger(log_queue=log_queue)
        self.logger.info("Starting Process!")
 
        self.root = root
        self.root.title("MQTT-JakaZu5s Controller")
        self.root.geometry("1100x1080")

        for col in range(8):
            self.root.grid_columnconfigure(col, weight=1, uniform="equal")
        
        row = 0
        self.button_ConnectRobot = \
            tk.Button(self.root, text="ConnectRobot", padx=5,
                      command=self.ConnectRobot, state="normal")
        self.button_ConnectRobot.grid(row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button_ConnectMQTT = \
            tk.Button(self.root, text="ConnectMQTT", padx=5,
                             command=self.ConnectMQTT, state="normal")
        self.button_ConnectMQTT.grid(row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button_DemoPutDownBox = \
            tk.Button(self.root, text="DemoPutDownBox", padx=5,
                       command=self.DemoPutDownBox, state="disabled")
        self.button_DemoPutDownBox.grid(row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        row += 1

        self.button_EnableRobot = \
            tk.Button(self.root, text="EnableRobot", padx=5,
                      command=self.EnableRobot, state="disabled")
        self.button_EnableRobot.grid(row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button_DisableRobot = \
            tk.Button(self.root, text="DisableRobot", padx=5,
                      command=self.DisableRobot, state="disabled")
        self.button_DisableRobot.grid(row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button_ReleaseHand = \
            tk.Button(self.root, text="ReleaseHand", padx=5,
                      command=self.ReleaseHand, state="disabled")
        self.button_ReleaseHand.grid(row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        self.frame_enabled = tk.Frame(self.root)
        self.frame_enabled.grid(row=row,column=6,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_enabled = \
            tk.Canvas(self.frame_enabled, width=10, height=10)
        self.canvas_enabled.pack(side="left",padx=10)
        self.light_enabled = \
            self.canvas_enabled.create_oval(1, 1, 9, 9, fill="gray")
        self.label_enabled = \
            tk.Label(self.frame_enabled, text="Enabled")
        self.label_enabled.pack(side="left",padx=2)

        row += 1

        self.button_DefaultPose = \
            tk.Button(self.root, text="DefaultPose", padx=5,
                      command=self.DefaultPose, state="disabled")
        self.button_DefaultPose.grid(row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button_TidyPose = \
            tk.Button(self.root, text="TidyPose", padx=5,
                      command=self.TidyPose, state="disabled")
        self.button_TidyPose.grid(row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button_ClearError = \
            tk.Button(self.root, text="ClearError", padx=5,
                      command=self.ClearError, state="disabled")
        self.button_ClearError.grid(row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        self.frame_error = tk.Frame(self.root)
        self.frame_error.grid(row=row,column=6,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_error = \
            tk.Canvas(self.frame_error, width=10, height=10)
        self.canvas_error.pack(side="left",padx=10)
        self.light_error = \
            self.canvas_error.create_oval(1, 1, 9, 9, fill="gray")
        self.label_error = \
            tk.Label(self.frame_error, text="Error")
        self.label_error.pack(side="left",padx=2)

        row += 1

        self.button_StartMQTTControl = \
            tk.Button(self.root, text="StartMQTTControl", padx=5,
                      command=self.StartMQTTControl, state="disabled")
        self.button_StartMQTTControl.grid(
            row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)
        
        self.button_StopMQTTControl = \
            tk.Button(self.root, text="StopMQTTControl", padx=5,
                      command=self.StopMQTTControl, state="disabled")
        self.button_StopMQTTControl.grid(
            row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button_ToolChange = \
            tk.Button(self.root, text="ToolChange", padx=5,
                      command=self.ToolChange, state="disabled")
        self.button_ToolChange.grid(
            row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        self.frame_mqtt_control = tk.Frame(self.root)
        self.frame_mqtt_control.grid(row=row,column=6,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_mqtt_control = \
            tk.Canvas(self.frame_mqtt_control, width=10, height=10)
        self.canvas_mqtt_control.pack(side="left",padx=10)
        self.light_mqtt_control = \
            self.canvas_mqtt_control.create_oval(1, 1, 9, 9, fill="gray")
        self.label_mqtt_control = \
            tk.Label(self.frame_mqtt_control, text="MQTTControl")
        self.label_mqtt_control.pack(side="left",padx=2)

        # 加速機能つきボタン
        class AccelerateButton(tk.Button):
            def __init__(
                self,
                master,
                jog_callback,
                jog_args,
                accelerate_settings=[
                    {"elapsed": 0, "step": 0.1, "interval": 100},
                    {"elapsed": 1, "step": 1, "interval": 250},
                ],
                **kwargs,
            ):
                super().__init__(master, **kwargs)
                self.jog_callback = jog_callback
                self.jog_args = jog_args
                self.accelerate_settings = accelerate_settings
                self._after_id = None
                self._interval = 100  # ms 初期間隔
                self._step = 1  # 初期増分
                self._press_time = None
                self.bind('<ButtonPress-1>', self._on_press)
                self.bind('<ButtonRelease-1>', self._on_release)
                self.bind('<Leave>', self._on_release)

            def _on_press(self, event=None):
                self._press_time = datetime.datetime.now()
                self._repeat()

            def _on_release(self, event=None):
                if self._after_id:
                    self.after_cancel(self._after_id)
                    self._after_id = None
                self._press_time = None

            def _repeat(self):
                # 経過時間で加速
                if self._press_time:
                    elapsed = (datetime.datetime.now() - self._press_time).total_seconds()
                    for setting in self.accelerate_settings:
                        if elapsed >= setting["elapsed"]:
                            self._step = setting["step"]
                            self._interval = setting["interval"]
                    # コールバック呼び出し
                    self.jog_callback(*self.jog_args, step=self._step)
                    self._after_id = self.after(self._interval, self._repeat)

        # Joint Jog
        row += 1
        tk.Label(self.root, text="Joint Jog").grid(row=row, column=0, padx=2, pady=2, sticky="w")
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        joint_accelerate_settings = [
            {"elapsed": 0, "step": 0.1, "interval": 100},
            {"elapsed": 1, "step": 1, "interval": 250},
        ]
        for i, joint in enumerate(joint_names):
            frame = tk.Frame(self.root)
            frame.grid(row=row, column=1+i, padx=2, pady=2, sticky="ew")
            tk.Label(frame, text=joint, width=2, anchor="e").pack(side="left", padx=10)
            btn_minus = AccelerateButton(frame, self.jog_joint_accel, (i, -1), 
                                         accelerate_settings=joint_accelerate_settings, text="-", width=2)
            btn_minus.pack(side="left", expand=True, fill="x")
            btn_plus = AccelerateButton(frame, self.jog_joint_accel, (i, 1),
                                         accelerate_settings=joint_accelerate_settings, text="+", width=2)
            btn_plus.pack(side="left", expand=True, fill="x")

        # TCP Jog
        row += 1
        tk.Label(self.root, text="TCP Jog").grid(row=row, column=0, padx=2, pady=2, sticky="w")
        tcp_names = ["X", "Y", "Z", "RX", "RY", "RZ"]
        tcp_accelerate_settings = [
            {"elapsed": 0, "step": 0.1, "interval": 100},
            {"elapsed": 1, "step": 1, "interval": 250},
            {"elapsed": 2, "step": 10, "interval": 500},
        ]
        for i, tcp in enumerate(tcp_names):
            frame = tk.Frame(self.root)
            frame.grid(row=row, column=1+i, padx=2, pady=2, sticky="ew")
            tk.Label(frame, text=tcp, width=2, anchor="e").pack(side="left", padx=10)
            btn_minus = AccelerateButton(frame, self.jog_tcp_accel, (i, -1),
                                         accelerate_settings=tcp_accelerate_settings, text="-", width=2)
            btn_minus.pack(side="left", expand=True, fill="x")
            btn_plus = AccelerateButton(frame, self.jog_tcp_accel, (i, 1),
                                         accelerate_settings=tcp_accelerate_settings, text="+", width=2)
            btn_plus.pack(side="left", expand=True, fill="x")

        row += 1

        tk.Label(self.root, text="State").grid(
            row=row, column=0, padx=2, pady=10, sticky="w", columnspan=4)
        self.string_var_states = {}
        for i in range(6):
            frame_state = tk.Frame(self.root)
            frame_state.grid(row=row+1+i, column=0, padx=2, pady=2, sticky="ew", columnspan=2)
            label_target = tk.Label(frame_state, text=f"J{i + 1}")
            label_target.pack(side="left", padx=10)
            string_var_state = tk.StringVar()
            string_var_state.set("")
            self.string_var_states[f"J{i + 1}"] = string_var_state
            text_box_state = tk.Label(
                frame_state,
                textvariable=string_var_state,
                bg="white",
                relief="solid",
                bd=1,
                anchor="e",
            )
            text_box_state.pack(side="right", padx=2, expand=True, fill="x")

        frame_state = tk.Frame(self.root)
        frame_state.grid(row=row+7, column=0, padx=2, pady=2, sticky="ew", columnspan=2)
        label_target = tk.Label(frame_state, text="Tool ID")
        label_target.pack(side="left", padx=10)
        string_var_state = tk.StringVar()
        string_var_state.set("")
        self.string_var_states["Tool ID"] = string_var_state
        text_box_state = tk.Label(
            frame_state,
            textvariable=string_var_state,
            bg="white",
            relief="solid",
            bd=1,
            anchor="e",
        )
        text_box_state.pack(side="right", padx=2, expand=True, fill="x")

        self.string_var_states_tcp = {}
        for i in range(6):
            frame_state = tk.Frame(self.root)
            frame_state.grid(row=row+1+i, column=2, padx=2, pady=2, sticky="ew", columnspan=2)
            label_target = tk.Label(frame_state, text=f"{tcp_names[i]}")
            label_target.pack(side="left", padx=10)
            string_var_state = tk.StringVar()
            string_var_state.set("")
            self.string_var_states_tcp[i] = string_var_state
            text_box_state = tk.Label(
                frame_state,
                textvariable=string_var_state,
                bg="white",
                relief="solid",
                bd=1,
                anchor="e",
            )
            text_box_state.pack(side="right", padx=2, expand=True, fill="x")

        tk.Label(self.root, text="Target").grid(
            row=row, column=4, padx=2, pady=2, sticky="w", columnspan=4)
        self.string_var_targets = {}
        for i in range(6):
            frame_target = tk.Frame(self.root)
            frame_target.grid(
                row=row+1+i, column=4, padx=2, pady=2, sticky="ew", columnspan=2)
            label_target = tk.Label(frame_target, text=f"J{i + 1}")
            label_target.pack(side="left", padx=10)
            string_var_target = tk.StringVar()
            string_var_target.set("")
            self.string_var_targets[f"J{i + 1}"] = string_var_target
            text_box_target = tk.Label(
                frame_target,
                textvariable=string_var_target,
                bg="white",
                relief="solid",
                bd=1,
                anchor="e",
            )
            text_box_target.pack(side="right", padx=2, expand=True, fill="x")

        frame_target = tk.Frame(self.root)
        frame_target.grid(row=row+7, column=4, padx=2, pady=2, sticky="ew", columnspan=2)
        label_target = tk.Label(frame_target, text="grip")
        label_target.pack(side="left", padx=10)
        string_var_target = tk.StringVar()
        string_var_target.set("")
        self.string_var_targets["grip"] = string_var_target
        text_box_target = tk.Label(
            frame_target,
            textvariable=string_var_target,
            bg="white",
            relief="solid",
            bd=1,
            anchor="e",
        )
        text_box_target.pack(side="right", padx=2, expand=True, fill="x")

        row += 8

        tk.Label(self.root, text="Topics").grid(
            row=row, column=0, padx=2, pady=10, sticky="w", columnspan=8)
        topic_types = [
            "mgr/register",
            "dev",
            "robot",
            "control",
        ]
        self.string_var_topics = {
            topic: tk.StringVar() for topic in topic_types}
        self.topic_monitors = {}
        for i, topic_type in enumerate(topic_types):
            frame_topic = tk.Frame(self.root)
            frame_topic.grid(
                row=row+1+3*i, column=0, padx=2, pady=2,
                sticky="ew", columnspan=8)
            label_topic_type = tk.Label(frame_topic, text=topic_type)
            label_topic_type.pack(side="left", padx=2)
            label_actual_topic = tk.Label(
                frame_topic, text="(Actual Topic)")
            label_actual_topic.pack(side="left", padx=2)
            string_var_topic = self.string_var_topics[topic_type]
            text_box_topic = tk.Label(
                frame_topic,
                textvariable=string_var_topic,
                bg="white",
                relief="solid",
                bd=1,
                anchor="w",
            )
            text_box_topic.pack(side="left", padx=2, expand=True, fill="x")
            frame_topic = tk.Frame(self.root)
            frame_topic.grid(
                row=row+2+3*i, column=0, padx=2, pady=2,
                sticky="ew", columnspan=8, rowspan=2)
            self.topic_monitors[topic_type] = scrolledtext.ScrolledText(
                frame_topic, height=3)
            self.topic_monitors[topic_type].pack(
                side="left", padx=2, expand=True, fill="both")

        row += 1 + 3*len(topic_types)

        frame_sm = tk.Frame(self.root)
        frame_sm.grid(
            row=row, column=0, padx=2, pady=10, sticky="ew", columnspan=8)
        label_sm = tk.Label(frame_sm, text="Shared Memory (rounded)")
        label_sm.pack(side="left", padx=2)
        self.string_var_sm = tk.StringVar()
        text_box_sm = tk.Label(
            frame_sm,
            textvariable=self.string_var_sm,
            bg="white",
            relief="solid",
            bd=1,
            anchor="w",
        )
        text_box_sm.pack(side="left", padx=2, expand=True, fill="x")

        row += 1

        tk.Label(self.root, text="Log Monitor").grid(
            row=row, column=0, padx=2, pady=2, sticky="w", columnspan=8)
        self.log_monitor = scrolledtext.ScrolledText(
            self.root, height=10)
        self.log_monitor.grid(
            row=row+1,column=0,padx=2,pady=2,columnspan=8, sticky="nsew")
        self.log_monitor.tag_config("INFO", foreground="black")
        self.log_monitor.tag_config("WARNING", foreground="orange")
        self.log_monitor.tag_config("ERROR", foreground="red")
        self.update_monitor()

    def setup_logging(
        self, log_queue: Optional[multiprocessing.Queue] = None,
    ) -> None:
        """複数プロセスからのログを集約する方法を設定する"""
        t_start_str = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        handlers = [
            logging.FileHandler(t_start_str + "_log.txt"),
            logging.StreamHandler(),
        ]
        if log_queue is not None:
            handlers.append(GUILoggingHandler(self))
        formatter = MicrosecondFormatter(
            "[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S.%f",
        )
        for handler in handlers:
            handler.setFormatter(formatter)
        self.listener = QueueListener(log_queue, *handlers)
        # listener.startからroot.mainloopまでのわずかな間は、
        # GUIの更新がないが、root.mainloopが始まると溜まっていたログも表示される
        self.listener.start()

    def setup_logger(
        self, log_queue: Optional[multiprocessing.Queue] = None,
    ) -> None:
        """GUIプロセスからのログの設定"""
        self.logger = logging.getLogger("GUI")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def ConnectRobot(self):
        if self.pm.state_control and self.pm.state_monitor:
            return
        self.pm.startControl()
        self.pm.startMonitor()
        if self.use_joint_monitor_plot:
            self.pm.startMonitorGUI()
        self.button_ConnectRobot.config(state="disabled")
        self.button_ClearError.config(state="normal")
        self.button_DefaultPose.config(state="disabled")
        self.button_DisableRobot.config(state="normal")
        self.button_EnableRobot.config(state="normal")
        self.button_ReleaseHand.config(state="normal")
        self.button_TidyPose.config(state="normal")
        self.button_ToolChange.config(state="disabled")
        if self.pm.state_recv_mqtt:
            self.button_StartMQTTControl.config(state="normal")
            self.button_StopMQTTControl.config(state="normal")

    def ConnectMQTT(self):
        if self.pm.state_recv_mqtt:
            return
        self.pm.startRecvMQTT()
        self.button_ConnectMQTT.config(state="disabled")
        self.button_ClearError.config(state="normal")
        self.button_DefaultPose.config(state="disabled")
        self.button_DisableRobot.config(state="normal")
        self.button_EnableRobot.config(state="normal")
        self.button_ReleaseHand.config(state="normal")
        self.button_TidyPose.config(state="normal")
        if self.pm.state_control and self.pm.state_monitor:
            self.button_StartMQTTControl.config(state="normal")
            self.button_StopMQTTControl.config(state="normal")

    def EnableRobot(self):
        if not self.pm.state_control:
            return
        self.pm.enable()

    def DisableRobot(self):
        if not self.pm.state_control:
            return
        self.pm.disable()

    def DefaultPose(self):
        if not self.pm.state_control:
            return
        self.pm.default_pose()
    
    def TidyPose(self):
        if not self.pm.state_control:
            return
        self.pm.tidy_pose()

    def ClearError(self):
        if not self.pm.state_control:
            return
        self.pm.clear_error()

    def StartMQTTControl(self):
        if ((not self.pm.state_control) or
            (not self.pm.state_monitor) or
            (not self.pm.state_recv_mqtt)):
            return
        self.pm.start_mqtt_control()

    def StopMQTTControl(self):
        if ((not self.pm.state_control) or
            (not self.pm.state_monitor) or
            (not self.pm.state_recv_mqtt)):
            return
        self.pm.stop_mqtt_control()

    def ReleaseHand(self):
        if not self.pm.state_control:
            return
        self.pm.release_hand()

    def ToolChange(self):
        pass
        # if not self.pm.state_control:
        #     return
        # popup = ToolChangePopup(self.root)
        # tool_id = popup.result
        # if tool_id is None:
        #     return
        # self.pm.tool_change(tool_id)

    def jog_joint(self, joint, direction):
        if not self.pm.state_control:
            return
        self.pm.jog_joint(joint, direction)

    def jog_tcp(self, axis, direction):
        if not self.pm.state_control:
            return
        self.pm.jog_tcp(axis, direction)

    def jog_joint_accel(self, joint, direction, step=1):
        # 加速対応のジョイントジョグコールバック
        self.jog_joint(joint, direction * step)

    def jog_tcp_accel(self, axis, direction, step=1):
        # 加速対応のTCPジョグコールバック
        self.jog_tcp(axis, direction * step)

    def DemoPutDownBox(self):
        pass
        # if not self.pm.state_control:
        #     return
        # self.pm.demo_put_down_box()

    def update_monitor(self):
        # モニタープロセスからの情報
        log = self.pm.get_current_monitor_log()
        if log:
            # ロボットの姿勢情報が流れるトピック
            topic_type = log.pop("topic_type")
            topic = log.pop("topic")
            poses = log.pop("poses", None)
            log_str = json.dumps(log, ensure_ascii=False)
            self.string_var_topics[topic_type].set(topic)
            self.update_topic(log_str, self.topic_monitors[topic_type])

            # 各情報をパース
            color = "lime" if log.get("enabled") else "gray"
            self.canvas_enabled.itemconfig(self.light_enabled, fill=color)
            color = "lime" if log.get("mqtt_control") == "ON" else "gray"
            self.canvas_mqtt_control.itemconfig(
                self.light_mqtt_control, fill=color)
            color = "red" if "error" in log else "gray"
            self.canvas_error.itemconfig(self.light_error, fill=color)
            joints = log.get("joints")
            if joints is not None:
                for i in range(6):
                    self.string_var_states[f"J{i + 1}"].set(f"{joints[i]:.2f}")
            else:
                for i in range(6):
                    self.string_var_states[f"J{i + 1}"].set("")
            tool_id = log.get("tool_id")
            if tool_id is not None:
                self.string_var_states["Tool ID"].set(f"{tool_id}")
            else:
                self.string_var_states["Tool ID"].set("")
            if poses is not None:
                for i in range(6):
                    self.string_var_states_tcp[i].set(f"{poses[i]:.2f}")

        # MQTT制御プロセスからの情報
        log = self.pm.get_current_mqtt_control_log()
        if log:
            topic_type = log.pop("topic_type")
            topic = log.pop("topic")
            log_str = json.dumps(log, ensure_ascii=False)
            self.string_var_topics[topic_type].set(topic)
            self.update_topic(log_str, self.topic_monitors[topic_type])

            joints = log.get("joints")
            if joints is not None:
                for i in range(6):
                    self.string_var_targets[f"J{i + 1}"].set(f"{joints[i]:.2f}")
            else:
                for i in range(6):
                    self.string_var_targets[f"J{i + 1}"].set("")
            grip = log.get("grip")
            if grip is not None:
                self.string_var_targets["grip"].set(f"{grip}")
            else:
                self.string_var_targets["grip"].set("")
        
        # 共有メモリの情報
        sm = self.pm.ar.copy()
        sm_str = ",".join(str(int(round(x))) for x in sm)
        self.string_var_sm.set(sm_str)
        self.root.after(100, self.update_monitor)  # 100ms間隔で表示を更新

    def update_topic(self, msg: str, box: scrolledtext.ScrolledText) -> None:
        """トピックの内容を表示する"""
        box.delete("1.0", tk.END)
        box.insert(tk.END, msg + "\n")  # ログを表示

    def update_log(self, msg: str) -> None:
        """ログを表示する"""
        box = self.log_monitor

        ## 最後の行を表示しているときだけ自動スクロールする
        # 挿入前にスクロールバーが一番下かどうかを判定
        # yview()は(最初に表示されている行, 最後に表示されている行)を0.0～1.0で返す
        at_bottom = False
        if box.yview()[1] >= 0.999:  # 浮動小数点の誤差を考慮
            at_bottom = True
        # ログレベルごとに色を変える
        if "[ERROR]" in msg:
            tag = "ERROR"
        elif "[WARNING]" in msg:
            tag = "WARNING"
        else:
            tag = "INFO"
        box.insert(tk.END, msg + "\n", tag)  # ログを表示
        # 挿入後、元々一番下にいた場合のみ自動スクロール
        if at_bottom:
            box.see(tk.END)
        
        # 行数が多すぎる場合は古い行を一部削除してメモリを節約
        current_lines = int(box.index('end-1c').split('.')[0])
        if current_lines > 1000:
            excess_lines = current_lines - 1000
            box.delete("1.0", f"{excess_lines}.0")
   
    def on_closing(self):
        """ウインドウを閉じるときの処理"""
        self.pm.stop_all_processes()
        time.sleep(1)
        self.root.destroy()
        self.handler.close()
        self.listener.stop()
        logging.shutdown()


if __name__ == '__main__':
    # Freeze Support for Windows
    multiprocessing.freeze_support()


    # NOTE: 現在ロボットに付いているツールが何かを管理する方法がないので
    # ロボット制御コードの使用者に指定してもらう
    # ツールによっては、ツールとの通信が不要なものがあるので、通信の成否では判定できない
    # 現在のツールの状態を常にファイルに保存しておき、ロボット制御コードを再起動するときに
    # そのファイルを読み込むようにすれば管理はできるが、エラーで終了したときに
    # ファイルの情報が正確かいまのところ保証できないので、指定してもらう
    import argparse
    parser = argparse.ArgumentParser()
    # parser.add_argument(
    #     "--tool-id",
    #     type=int,
    #     required=True,
    #     choices=tool_ids,
    #     help="現在ロボットに付いているツールのID",
    # )
    parser.add_argument(
        "--use-joint-monitor-plot",
        action="store_true",
        help="関節角度のモニタープロットを使用する",
    )
    args = parser.parse_args()
    kwargs = vars(args)
    import os
    # HACK: コードの変化を少なくするため、
    # ロボット制御プロセスに引数で渡すのではなく環境変数で渡す
    # os.environ["TOOL_ID"] = str(kwargs.pop("tool_id"))
    os.environ["TOOL_ID"] = "1"

    root = tk.Tk()
    mqwin = MQTTWin(root, **kwargs)
    mqwin.root.lift()
    root.protocol("WM_DELETE_WINDOW", mqwin.on_closing)
    root.mainloop()
