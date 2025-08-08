import random
import time
from jaka_control.ag95_extension import ExtendedAG95
from multiprocessing import Process, Value, Manager
import ctypes
import argparse
import tkinter as tk
import threading
import traceback


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


def control_loop(theta_tool, time_dict, simulate_error=False):
    gripper = ExtendedAG95(port='/dev/ttyUSB0')
    stop_event = threading.Event()
    error_flag = threading.Event()
    error_info = {}

    def worker():
        while not stop_event.is_set():
            with theta_tool.get_lock():
                tool_corrected = int(theta_tool.value)
            tool_corrected = (tool_corrected - (-1)) / (89 - (-1)) * (1000 - 0)
            tool_corrected = max(min(int(round(tool_corrected)), 1000), 0)
            t0 = time.perf_counter()
            try:
                gripper.set_pos(tool_corrected)
                t1 = time.perf_counter()
                random_value = random.randint(0, 1000) if simulate_error else 0
                time_dict['random_value'] = random_value
                if random_value == 1:
                    raise Exception("Simulated random error for testing inside worker.")
                time_dict['set_pos'] = t1 - t0
            except Exception as e:
                err_str = f"[ERROR][set_pos] {e}\n" + traceback.format_exc()
                time_dict['set_pos'] = 0.0
                print(err_str)
                error_info['msg'] = err_str
                error_flag.set()
                stop_event.set()
                break

            t2 = time.perf_counter()
            try:
                read_pos = gripper.read_pos()
                t3 = time.perf_counter()
                time_dict['read_pos'] = t3 - t2
                time_dict['read_pos_value'] = read_pos
            except Exception as e:
                err_str = f"[ERROR][read_pos] {e}\n" + traceback.format_exc()
                time_dict['read_pos'] = 0.0
                time_dict['read_pos_value'] = None
                print(err_str)
                error_info['msg'] = err_str
                error_flag.set()
                stop_event.set()
                break

    thread = threading.Thread(target=worker)
    thread.start()

    while True:
        if not thread.is_alive():
            break
        if error_flag.is_set():
            stop_event.set()
            break
        loop_start = time.perf_counter()
        try:
            random_value_main = random.randint(0, 1000) if simulate_error else 0
            time_dict['random_value_main'] = random_value_main
            if random_value_main == 1:
                raise Exception("Simulated random error for testing in main loop.")
            time.sleep(0.008)  # 8ms
            loop_end = time.perf_counter()
            time_dict['main_loop'] = loop_end - loop_start
        except Exception as e:
            err_str = f"[ERROR][control_loop] {e}\n" + traceback.format_exc()
            time_dict['main_loop'] = 0.0
            print(err_str)
            error_info['msg'] = err_str
            error_flag.set()
            stop_event.set()
            break
    thread.join()
    if error_flag.is_set():
        raise Exception(error_info['msg'])


def control_loop_with_automatic_recover(theta_tool, time_dict, simulate_error=False):
    while True:
        try:
            control_loop(theta_tool, time_dict, simulate_error)
        except Exception as e:
            err_str = f"[ERROR][control_loop_with_automatic_recover] {e}\n" + traceback.format_exc()
            print(err_str)
            time_dict['last_error'] = err_str
            time.sleep(1)


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
    tk.Label(root, text='random_value').pack()
    random_value_box = tk.Entry(root, width=20)
    random_value_box.pack()
    tk.Label(root, text='random_value_main').pack()
    random_value_main_box = tk.Entry(root, width=20)
    random_value_main_box.pack()
    tk.Label(root, text='last error').pack()
    last_error_box = tk.Text(root, width=60, height=6)
    last_error_box.pack()
    set_pos_box.insert(0, '0.0')
    read_pos_box.insert(0, '0.0')
    main_loop_box.insert(0, '0.0')
    read_pos_value_box.insert(0, 'None')
    random_value_box.insert(0, 'None')
    random_value_main_box.insert(0, 'None')
    last_error_box.insert('1.0', '')

    def update_boxes():
        set_time = time_dict.get('set_pos', 0.0)
        read_time = time_dict.get('read_pos', 0.0)
        main_loop_time = time_dict.get('main_loop', 0.0)
        read_pos_value = time_dict.get('read_pos_value', None)
        random_value = time_dict.get('random_value', None)
        random_value_main = time_dict.get('random_value_main', None)
        last_error = time_dict.get('last_error', '')
        set_pos_box.delete(0, tk.END)
        set_pos_box.insert(0, f'{set_time:.6f}')
        read_pos_box.delete(0, tk.END)
        read_pos_box.insert(0, f'{read_time:.6f}')
        main_loop_box.delete(0, tk.END)
        main_loop_box.insert(0, f'{main_loop_time:.6f}')
        read_pos_value_box.delete(0, tk.END)
        read_pos_value_box.insert(0, str(read_pos_value))
        random_value_box.delete(0, tk.END)
        random_value_box.insert(0, str(random_value))
        random_value_main_box.delete(0, tk.END)
        random_value_main_box.insert(0, str(random_value_main))
        last_error_box.delete('1.0', tk.END)
        last_error_box.insert('1.0', last_error)
        root.after(50, update_boxes)  # 20Hz更新

    update_boxes()
    root.mainloop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulate-error', action='store_true', help='エラー発生をシミュレートする')
    args = parser.parse_args()
    theta_tool = Value(ctypes.c_double, 89)
    manager = Manager()
    time_dict = manager.dict()
    p1 = Process(target=theta_updater, args=(theta_tool,))
    p2 = Process(target=control_loop_with_automatic_recover, args=(theta_tool, time_dict, args.simulate_error))
    p1.start()
    p2.start()
    try:
        gui_loop(time_dict)
    except KeyboardInterrupt:
        pass
    finally:
        p1.terminate()
        p2.terminate()
