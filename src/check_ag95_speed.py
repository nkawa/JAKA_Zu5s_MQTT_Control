import time
from pyDHgripper import AG95
from multiprocessing import Process, Value, Manager
import ctypes
import argparse
import tkinter as tk
import threading


def theta_updater(theta_tool):
    # 初期値を設定
    time.sleep(3)
    direction = 1
    while True:
        with theta_tool.get_lock():
            theta_tool.value += 0.5 * direction
            if theta_tool.value >= 89:
                theta_tool.value = 89
                direction = -1
            elif theta_tool.value <= -1:
                theta_tool.value = -1
                direction = 1
        time.sleep(0.0165)  # 16.5ms


def gripper_setter(theta_tool, interval, time_dict):
    gripper = AG95(port='/dev/ttyUSB0')
    stop_event = threading.Event()

    def worker():
        while not stop_event.is_set():
            with theta_tool.get_lock():
                tool_corrected = int(theta_tool.value)
            tool_corrected = (tool_corrected - (-1)) / (89 - (-1)) * (1000 - 0)
            tool_corrected = max(min(int(round(tool_corrected)), 1000), 0)
            t0 = time.perf_counter()
            try:
                gripper.set_pos(tool_corrected)
            except Exception as e:
                print(f"[ERROR][set_pos] {e}")
            t1 = time.perf_counter()
            time_dict['set_pos'] = t1 - t0

            t2 = time.perf_counter()
            try:
                read_pos = gripper.read_pos()
                t3 = time.perf_counter()
                time_dict['read_pos'] = t3 - t2
                time_dict['read_pos_value'] = read_pos
            except Exception as e:
                time_dict['read_pos'] = 0.0
                time_dict['read_pos_value'] = None
                print(f"[ERROR][read_pos] {e}")

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()

    while True:
        loop_start = time.perf_counter()
        time.sleep(interval)
        loop_end = time.perf_counter()
        time_dict['main_loop'] = loop_end - loop_start


def gui_loop(time_dict):
    root = tk.Tk()
    root.title('Gripper Timing Monitor')
    tk.Label(root, text='set_pos time [s]').pack()
    set_pos_box = tk.Entry(root, width=20)
    set_pos_box.pack()
    tk.Label(root, text='read_pos time [s]').pack()
    read_pos_box = tk.Entry(root, width=20)
    read_pos_box.pack()
    tk.Label(root, text='main_loop time [s]').pack()
    main_loop_box = tk.Entry(root, width=20)
    main_loop_box.pack()
    tk.Label(root, text='read_pos value').pack()
    read_pos_value_box = tk.Entry(root, width=20)
    read_pos_value_box.pack()
    set_pos_box.insert(0, '0.0')
    read_pos_box.insert(0, '0.0')
    main_loop_box.insert(0, '0.0')
    read_pos_value_box.insert(0, 'None')

    def update_boxes():
        set_time = time_dict.get('set_pos', 0.0)
        read_time = time_dict.get('read_pos', 0.0)
        main_loop_time = time_dict.get('main_loop', 0.0)
        read_pos_value = time_dict.get('read_pos_value', None)
        set_pos_box.delete(0, tk.END)
        set_pos_box.insert(0, f'{set_time:.6f}')
        read_pos_box.delete(0, tk.END)
        read_pos_box.insert(0, f'{read_time:.6f}')
        main_loop_box.delete(0, tk.END)
        main_loop_box.insert(0, f'{main_loop_time:.6f}')
        read_pos_value_box.delete(0, tk.END)
        read_pos_value_box.insert(0, str(read_pos_value))
        root.after(50, update_boxes)  # 20Hz更新

    update_boxes()
    root.mainloop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--interval', type=float, default=0.008, help='gripper_setter送信間隔[秒]')
    args = parser.parse_args()
    theta_tool = Value(ctypes.c_double, 89)
    manager = Manager()
    time_dict = manager.dict()
    p1 = Process(target=theta_updater, args=(theta_tool,))
    p2 = Process(target=gripper_setter, args=(theta_tool, args.interval, time_dict))
    p1.start()
    p2.start()
    try:
        gui_loop(time_dict)
    except KeyboardInterrupt:
        pass
    finally:
        p1.terminate()
        p2.terminate()
