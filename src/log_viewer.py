from functools import partial
import re
import sys
import os
import glob
import json
import pandas as pd  # データフレーム操作・時系列変換用
from PyQt6 import QtWidgets, QtCore, QtGui  # PyQt6: GUI構築用
import time
import logging

# pyqtgraph: 高速・インタラクティブなプロットライブラリ
import pyqtgraph as pg
from pyqtgraph import AxisItem
import numpy as np  # 数値配列処理用

# configure module-level logger
logger = logging.getLogger('log_viewer')
if not logger.handlers:
    logger.setLevel(logging.DEBUG)
    try:
        log_path = os.path.join(os.path.dirname(__file__), 'log_viewer_debug.log')
    except Exception:
        log_path = 'log_viewer_debug.log'
    fh = logging.FileHandler(log_path, encoding='utf-8')
    fmt = logging.Formatter('[%(asctime)s][%(levelname)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    fh.setFormatter(fmt)
    logger.addHandler(fh)

# try import tsdownsample (MinMaxLTTBDownSampler). 無ければフォールバック
try:
    from tsdownsample import MinMaxLTTBDownSampler
    HAS_TSD = True
    logger.debug('tsdownsample available: MinMaxLTTBDownSampler will be used when needed')
except Exception:
    HAS_TSD = False
    logger.debug('tsdownsample not available: falling back to uniform decimation')


class LogLoader:
    """
    ログファイルの読み込み・パースを担当するクラス。
    state.jsonl, control.jsonl, log.txt をDataFrameやリストで返す。
    """
    def __init__(self, log_dir):
        self.log_dir = log_dir

    def list_dates(self):
        # log/配下の日付ディレクトリ一覧を返す
        return sorted([d for d in os.listdir(self.log_dir) if os.path.isdir(os.path.join(self.log_dir, d))])

    def list_times(self, date):
        # log/日付/配下の時刻ディレクトリ一覧を返す
        date_dir = os.path.join(self.log_dir, date)
        return sorted([d for d in os.listdir(date_dir) if os.path.isdir(os.path.join(date_dir, d))])

    def load_state(self, date, time_str) -> dict[str, pd.DataFrame]:
        # state.jsonlを1行1レコードのJSON Linesとして読み込み、DataFrame化
        path = os.path.join(self.log_dir, date, time_str, 'state.jsonl')
        logger.info(f"load_state start: date={date} time={time_str} path={path}")
        t0 = time.perf_counter()
        if not os.path.exists(path):
            logger.info(f"load_state missing: {path}")
            return {}
        records = []
        with open(path) as f:
            for line in f:
                js = json.loads(line)
                data = {}
                data["time"] = js["time"]
                data["kind"] = js["kind"]
                for i in range(6):
                    data[f"J{i+1}"] = js["joint"][i]
                pose_names = ["X", "Y", "Z", "RX", "RY", "RZ"]
                for i in range(6):
                    data[pose_names[i]] = js["pose"][i]
                records.append(data)
        df = pd.DataFrame(records)
        ret = {}
        if df.empty:
            logger.info(f"load_state done: found 0 records (elapsed={time.perf_counter()-t0:.3f}s)")
            return ret
        for kind in df["kind"].unique():
            ret[kind] = df[df["kind"] == kind]
        logger.info(f"load_state done: records={len(df)} kinds={len(ret)} elapsed={time.perf_counter()-t0:.3f}s")
        return ret

    def load_control(self, date, time_str) -> dict[str, pd.DataFrame]:
        # control.jsonlも同様にDataFrame化
        path = os.path.join(self.log_dir, date, time_str, 'control.jsonl')
        logger.info(f"load_control start: date={date} time={time_str} path={path}")
        t0 = time.perf_counter()
        if not os.path.exists(path):
            logger.info(f"load_control missing: {path}")
            return {}
        records = []
        with open(path) as f:
            for line in f:
                js = json.loads(line)
                data = {}
                data["time"] = js["time"]
                data["kind"] = js["kind"]
                for i in range(6):
                    data[f"J{i+1}"] = js["joint"][i]
                data["max_ratio"] = js.get("max_ratio", None)
                data["accel_max_ratio"] = js.get("accel_max_ratio", None)
                records.append(data)
        df = pd.DataFrame(records)
        ret = {}
        if df.empty:
            logger.info(f"load_control done: found 0 records (elapsed={time.perf_counter()-t0:.3f}s)")
            return ret
        for kind in df["kind"].unique():
            ret[kind] = df[df["kind"] == kind]
        logger.info(f"load_control done: records={len(df)} kinds={len(ret)} elapsed={time.perf_counter()-t0:.3f}s")
        return ret

    def load_events(self, date, time_str):
        # log.txtを1行ずつリストで返す
        path = os.path.join(self.log_dir, date, time_str, 'log.txt')
        logger.info(f"load_events start: date={date} time={time_str} path={path}")
        t0 = time.perf_counter()
        if not os.path.exists(path):
            logger.info(f"load_events missing: {path}")
            return None
        events = []
        match = None
        dt_str, module, level, message, original_message = \
            None, None, None, None, None
        with open(path) as f:
            for line in f:
                # 各ログの形式は、[日時][モジュール][レベル] メッセージ
                # (メッセージは複数行になることがある)
                match = re.match(r'\[(.*?)\]\[(.*?)\]\[(.*?)\] (.*)', line)
                # 各ログの先頭行
                if match:
                    # ファイル最初のログ行への対応
                    if (
                        (dt_str is not None) and
                        (module is not None) and
                        (level is not None) and
                        (message is not None) and
                        (original_message is not None)
                    ):
                        events.append({
                            'time': pd.to_datetime(dt_str).tz_localize(
                                'Asia/Tokyo').timestamp(),
                            'module': module,
                            'level': level,
                            'message': message.strip(),
                            'original_message': original_message.strip(),
                            'is_event': 1,
                        })
                    dt_str, module, level, message = match.groups()
                    original_message = line
                # ログの続き行
                else:
                    if message is not None:
                        message += line
                    if original_message is not None:
                        original_message += line
            # ファイル最後のログ行への対応
            if match:
                if (
                    (dt_str is not None) and
                    (module is not None) and
                    (level is not None) and
                    (message is not None) and
                    (original_message is not None)
                ):
                    events.append({
                        'time': pd.to_datetime(dt_str).tz_localize(
                            'Asia/Tokyo').timestamp(),
                        'module': module,
                        'level': level,
                        'message': message.strip(),
                        'original_message': original_message.strip(),
                        'is_event': 1,
                    })

        logger.info(f"load_events done: events={len(events)} elapsed={time.perf_counter()-t0:.3f}s")
        return pd.DataFrame(events)

class LogViewer(QtWidgets.QWidget):
    def __init__(self, log_dir):
        super().__init__()
        self.loader = LogLoader(log_dir)
        self.init_ui()
        # ウィンドウのデフォルトサイズを大きくする（8行1列のプロットが見やすいように）
        self.resize(800, 1000)

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()
        # 日付・時刻選択UI
        date_label = QtWidgets.QLabel('日付:')
        self.date_combo = QtWidgets.QComboBox()
        self.date_combo.addItems(self.loader.list_dates())  # ログ日付一覧
        self.date_combo.currentTextChanged.connect(self.update_times)
        time_label = QtWidgets.QLabel('時刻:')
        self.time_combo = QtWidgets.QComboBox()
        self.time_combo.currentTextChanged.connect(self.load_data)
        hlayout = QtWidgets.QHBoxLayout()
        hlayout.addWidget(date_label)
        hlayout.addWidget(self.date_combo)
        hlayout.addWidget(time_label)
        hlayout.addWidget(self.time_combo)
        layout.addLayout(hlayout)
        # スケールリセットボタン
        self.reset_btn = QtWidgets.QPushButton('スケールリセット')
        self.reset_btn.clicked.connect(self.reset_view)
        layout.addWidget(self.reset_btn)
        # プロット設定
        # base variables plus per-joint max/accel ratio options
        base_vars = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'X', 'Y', 'Z', 'RX', 'RY', 'RZ']
        per_joint_suffixes = ['_max_ratio', '_accel_max_ratio']
        joint_extra = []
        for j in range(1, 7):
            for s in per_joint_suffixes:
                joint_extra.append(f'J{j}{s}')
        self.all_varnames = base_vars + joint_extra + ['Event']
        self.default_varnames = \
            ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'X', 'Event']
        class DateAxisItem(pg.AxisItem):
            # x軸を日時表示にするAxisItem
            def tickStrings(self, values, scale, spacing):
                # spacing: tick間隔（秒）
                labels = []
                # 1日=86400秒, 1時間=3600秒
                for i, v in enumerate(values):
                    dt = pd.to_datetime(float(v), unit='s', utc=True).tz_convert('Asia/Tokyo')
                    if spacing >= 86400:  # 1日以上の間隔なら日付のみ
                        label = dt.strftime('%Y-%m-%d')
                    elif spacing >= 3600:  # 1時間以上なら時刻（最左だけ日付＋時刻）
                        if i == 0:
                            label = dt.strftime('%Y-%m-%d %H:%M')
                        else:
                            label = dt.strftime('%H:%M')
                    elif spacing >= 60:  # 1分以上なら時:分
                        if i == 0:
                            label = dt.strftime('%Y-%m-%d %H:%M')
                        else:
                            label = dt.strftime('%H:%M')
                    else:  # 1分未満なら時:分:秒
                        if i == 0:
                            label = dt.strftime('%Y-%m-%d %H:%M:%S')
                        else:
                            label = dt.strftime('%H:%M:%S')
                    labels.append(label)
                return labels

        class FixedDigitsAxisItem(pg.AxisItem):
            def tickStrings(self, values, scale, spacing):
                # 指数表記で全体6桁程度に揃える（例: 1.23e+03）
                return [f"{v:+8.2e}" for v in values]

        # プロットエリア
        self.combos = []
        self.plots = []
        # (panel_index, series) -> PlotDataItem / ScatterPlotItem
        self.plot_items = {}
        # GC回避用
        self.proxies = []
        for i in range(8):
            # レイアウト
            row_layout = QtWidgets.QHBoxLayout()
            combo = QtWidgets.QComboBox()
            plot = pg.PlotWidget()
            row_layout.addWidget(combo)
            row_layout.addWidget(plot, stretch=1)
            layout.addLayout(row_layout)
            # Combo
            combo.currentTextChanged.connect(self.update_curve)
            self.combos.append(combo)
            # Plot
            axis_bottom = DateAxisItem(orientation='bottom')
            axis_left = FixedDigitsAxisItem(orientation='left')
            plot.setAxisItems({'bottom': axis_bottom, 'left': axis_left})
            plot.setBackground('w')
            plot.showGrid(x=True, y=True, alpha=0.3)
            # y軸ラベルの幅は自動調整・小さめフォント
            axis_left.setStyle(autoExpandTextSpace=True)
            axis_left.setTickFont(QtGui.QFont("IPAGothic", 8))
            # x軸ラベルの重なり防止: フォントサイズを小さく、autoExpandTextSpace有効化
            axis_bottom.setStyle(autoExpandTextSpace=True)
            axis_bottom.setTickFont(QtGui.QFont("IPAGothic", 8))
            # 軸・ラベル・グリッド・テキストを黒に
            plot.getAxis('left').setPen(pg.mkPen('k'))
            plot.getAxis('bottom').setPen(pg.mkPen('k'))
            plot.getAxis('left').setTextPen(pg.mkPen('k'))
            plot.getAxis('bottom').setTextPen(pg.mkPen('k'))
            plot.addLegend(colCount=3, offset=(0, 1), anchor=(1, 0))
            if i > 0:
                plot.setXLink(self.plots[0])
            self.plots.append(plot)
            # マウスイベント
            proxy = pg.SignalProxy(
                plot.scene().sigMouseMoved,
                rateLimit=60,
                slot=partial(self.on_hover, i),
            )
            self.proxies.append(proxy)

        self.setLayout(layout)
        self.update_times(self.date_combo.currentText())

        self.max_visible_points = 100000  # 表示レンジごとに最大点数
        # 拡大縮小・パン操作のたびにプロットするのは重いので、最初に操作してから
        # 150msの間の操作の結果をまとめて1回だけ反映させる (デバウンス処理)
        self.range_debounce_timer = QtCore.QTimer(self)
        self.range_debounce_timer.setSingleShot(True)  # 1回だけTimer.start()を呼ぶ
        self.range_debounce_timer.setInterval(150)  # ms, 適宜調整
        self.range_debounce_timer.timeout.connect(self._on_range_debounced)
        # ViewBox の X レンジ変化は plots[0] のみ監視で良い
        vb0 = self.plots[0].getViewBox()
        vb0.sigXRangeChanged.connect(lambda vb, rng: self.range_debounce_timer.start())
        logger.info("UI initialized")

    def update_times(self, date):
        # 日付が選択されたとき、時刻リストを更新
        self.time_combo.clear()
        self.time_combo.addItems(self.loader.list_times(date))
        if self.time_combo.count() > 0:
            self.load_data(self.time_combo.currentText())

    def load_data(self, time_str):
        # 日付・時刻で指定されたログファイルを読み込み、8行1列のプロットを生成
        date = self.date_combo.currentText()
        time_str = self.time_combo.currentText()
        logger.info(f"load_data start: date={date} time={time_str}")
        t0 = time.perf_counter()
        state_dfs = self.loader.load_state(date, time_str)
        control_dfs = self.loader.load_control(date, time_str)
        self.target_df = control_dfs.get("target")
        self.control_df = control_dfs.get("control")
        self.state_df = state_dfs.get("state")
        self.event_df = self.loader.load_events(date, time_str)
        logger.info(f"load_data finished: state_rows={0 if self.state_df is None else len(self.state_df)} control_rows={0 if self.control_df is None else len(self.control_df)} event_rows={0 if self.event_df is None else len(self.event_df)} elapsed={time.perf_counter()-t0:.3f}s")
        for i_panel in range(8):
            plot = self.plots[i_panel]
            plot.clear()
            combo = self.combos[i_panel]
            combo.clear()
            combo.addItems(self.all_varnames)
            varname = self.default_varnames[i_panel]
            i_varname = self.all_varnames.index(varname)
            combo.setCurrentIndex(i_varname)
        self.plot_items.clear()
        self.reset_view()

    def update_curve(self):
        # リストボックスで選択された要素に応じてプロットを更新
        combo = self.sender()
        if combo is None:
            return
        idx = self.combos.index(combo)
        varname = combo.currentText()
        plot = self.plots[idx]
        # 既存のプロットをクリア
        plot.clear()
        # remove cache for this panel
        keys_to_remove = [k for k in list(self.plot_items.keys()) if k[0] == idx]
        for k in keys_to_remove:
            del self.plot_items[k]
        self.plot(varname, plot)

    def plot(self, varname, plot):
        # get panel index for storing items
        try:
            idx = self.plots.index(plot)
        except ValueError:
            idx = None

        logger.debug(f"plot start: varname={varname} idx={idx}")
        if varname == 'Event':
            if self.event_df is not None:
                # Use PlotDataItem groups of vertical segments (NaN-separated) for performance
                t_start = time.perf_counter()
                ts = self.event_df["time"].values
                levels = self.event_df["level"].values
                # map levels to colors
                level_to_color = {
                    'DEBUG': 'black',
                    'INFO': 'black',
                    'WARNING': 'orange',
                    'ERROR': 'red',
                    'CRITICAL': 'red',
                }
                if len(ts) > 0:
                    y_bottom = 0.0
                    y_top = 1.0
                    event_items = {}
                    # group times by color to create one PlotDataItem per color (fewer items, per-color pen)
                    colors = {}
                    for t, level in zip(ts, levels):
                        c = level_to_color.get(level, 'k')
                        colors.setdefault(c, []).append(t)
                    for color, t_list in colors.items():
                        m = len(t_list)
                        xs = np.empty(3 * m)
                        ys = np.empty(3 * m)
                        xs[0::3] = t_list
                        xs[1::3] = t_list
                        xs[2::3] = np.nan
                        ys[0::3] = y_bottom
                        ys[1::3] = y_top
                        ys[2::3] = np.nan
                        item = pg.PlotDataItem(xs, ys, pen=pg.mkPen(color, width=1), name='event')
                        plot.addItem(item)
                        event_items[color] = item
                    if idx is not None:
                        # store dict of items for this panel's events
                        self.plot_items[(idx, 'event')] = event_items
                    logger.info(f"plot Event: created colors={len(event_items)} total_events={len(ts)} elapsed={time.perf_counter()-t_start:.3f}s")
                # y軸範囲を0-1に固定
                plot.setYRange(0, 1)
        else:
            # For target/control/state, support direct columns and joint-specific
            # synthetic names like 'J1_max_ratio' / 'J2_accel_max_ratio'. Use
            # _get_ts_y_for_var to extract (ts,y) arrays and downsample to the
            # current view range to avoid plotting huge arrays initially.
            # determine current x-range from the plot's ViewBox if available
            try:
                vb = plot.getViewBox()
                t0, t1 = vb.viewRange()[0]
            except Exception:
                t0, t1 = None, None

            for series, df, color in (('target', self.target_df, 'r'),
                                      ('control', self.control_df, 'g'),
                                      ('state',  self.state_df,  'b')):
                ts, y = self._get_ts_y_for_var(df, varname)
                if ts is None or y is None:
                    # create empty placeholder so UI clears correctly
                    item = plot.plot([], [], pen=pg.mkPen(color, width=1), name=series)
                    if idx is not None:
                        self.plot_items[(idx, series)] = item
                    continue
                # if we have a view range, downsample to it; otherwise downsample full
                if t0 is None or t1 is None:
                    ds_ts, ds_y = self._downsample_visible_range(ts, y, ts.min() if len(ts) else 0, ts.max() if len(ts) else 0, self.max_visible_points)
                else:
                    ds_ts, ds_y = self._downsample_visible_range(ts, y, t0, t1, self.max_visible_points)

                item = plot.plot(ds_ts, ds_y, pen=pg.mkPen(color, width=1), name=series)
                logger.info(f"plot series: var={varname} series={series} original={len(ts)} plotted={len(ds_ts)}")
                if idx is not None:
                    self.plot_items[(idx, series)] = item

            # Also ensure Event creation only includes visible events (avoid plotting all at once)
            # If current combo is not Event this branch won't be executed during normal plotting,
            # but protect in case plot() called with Event earlier.
            if varname == 'Event' and self.event_df is not None:
                try:
                    vb = plot.getViewBox()
                    t0_ev, t1_ev = vb.viewRange()[0]
                except Exception:
                    t0_ev, t1_ev = None, None
                ts_all = self.event_df['time'].values
                levels_all = self.event_df['level'].values
                if t0_ev is None or t1_ev is None:
                    mask = np.ones(len(ts_all), dtype=bool)
                else:
                    mask = (ts_all >= t0_ev) & (ts_all <= t1_ev)
                ts_vis = ts_all[mask]
                levels_vis = levels_all[mask]
                if len(ts_vis) > 0:
                    y_bottom = 0.0
                    y_top = 1.0
                    event_items = {}
                    colors = {}
                    level_to_color = {
                        'DEBUG': 'black',
                        'INFO': 'black',
                        'WARNING': 'orange',
                        'ERROR': 'red',
                        'CRITICAL': 'red',
                    }
                    for t, level in zip(ts_vis, levels_vis):
                        c = level_to_color.get(level, 'k')
                        colors.setdefault(c, []).append(t)
                    for color, t_list in colors.items():
                        m = len(t_list)
                        xs = np.empty(3 * m)
                        ys = np.empty(3 * m)
                        xs[0::3] = t_list
                        xs[1::3] = t_list
                        xs[2::3] = np.nan
                        ys[0::3] = y_bottom
                        ys[1::3] = y_top
                        ys[2::3] = np.nan
                        item = pg.PlotDataItem(xs, ys, pen=pg.mkPen(color, width=1), name='event')
                        plot.addItem(item)
                        event_items[color] = item
                    if idx is not None:
                        self.plot_items[(idx, 'event')] = event_items

    def _downsample_visible_range(self, ts, y, t0, t1, max_points):
        # ts, y are numpy arrays (timestamps in seconds)
        if ts is None or len(ts) == 0:
            return np.array([]), np.array([])
        tstart = time.perf_counter()
        # ensure numpy arrays
        ts = np.asarray(ts)
        y = np.asarray(y)
        i = np.searchsorted(ts, t0, side='left')
        j = np.searchsorted(ts, t1, side='right')
        ts_vis = ts[i:j]
        y_vis = y[i:j]
        n = len(ts_vis)
        if n <= max_points:
            logger.debug(f"_downsample_visible_range: n={n} <= max_points={max_points} elapsed={time.perf_counter()-tstart:.6f}s")
            return ts_vis, y_vis
        # try tsdownsample
        if HAS_TSD:
            try:
                sampler = MinMaxLTTBDownSampler()
                idx = sampler.downsample(ts_vis, y_vis, n_out=max_points)
                logger.debug(f"_downsample_visible_range: sampled {n} -> {len(idx)} elapsed={time.perf_counter()-tstart:.6f}s")
                return ts_vis[idx], y_vis[idx]
            except Exception:
                # fallback to uniform decimation
                idx = np.linspace(0, n - 1, max_points).astype(int)
                logger.debug(f"_downsample_visible_range: fallback decimation elapsed={time.perf_counter()-tstart:.6f}s")
                return ts_vis[idx], y_vis[idx]
        else:
            idx = np.linspace(0, n - 1, max_points).astype(int)
            logger.debug(f"_downsample_visible_range: decimated {n} -> {len(idx)} elapsed={time.perf_counter()-tstart:.6f}s")
            return ts_vis[idx], y_vis[idx]

    def _on_range_debounced(self):
        logger.info("_on_range_debounced start")
        t0_global = time.perf_counter()
        # Called in UI thread after debounce interval
        if len(self.plots) == 0:
            logger.info("_on_range_debounced: no plots")
            return
        vb = self.plots[0].getViewBox()
        t0, t1 = vb.viewRange()[0]
        for i_panel, plot in enumerate(self.plots):
            # current var
            try:
                varname = self.combos[i_panel].currentText()
            except Exception:
                varname = None
            if varname is not None and varname != 'Event':
                for series, df, color in (('target', self.target_df, 'r'),
                                           ('control', self.control_df, 'g'),
                                           ('state',  self.state_df,  'b')):
                    item = self.plot_items.get((i_panel, series))
                    ts, y = self._get_ts_y_for_var(df, varname)
                    if ts is None or y is None:
                        if item is not None:
                            try:
                                item.setData([], [])
                            except Exception:
                                pass
                        continue
                    ds_ts, ds_y = self._downsample_visible_range(ts, y, t0, t1, self.max_visible_points)
                    if item is None:
                        pen = pg.mkPen(color, width=1)
                        new_item = pg.PlotDataItem(ds_ts, ds_y, pen=pen, name=series)
                        plot.addItem(new_item)
                        self.plot_items[(i_panel, series)] = new_item
                    else:
                        try:
                            item.setData(ds_ts, ds_y)
                        except Exception:
                            try:
                                item.setData(x=ds_ts, y=ds_y)
                            except Exception:
                                pass
            # Events: update visible vertical-line groups if present
            if varname == 'Event' and getattr(self, 'event_df', None) is not None:
                ev = self.event_df
                if ev is None or len(ev) == 0:
                    continue
                xs_all = ev['time'].values
                levels_all = ev['level'].values
                mask = (xs_all >= t0) & (xs_all <= t1)
                xs_vis = xs_all[mask]
                levels_vis = levels_all[mask]
                y_bottom = 0.0
                y_top = 1.0
                # same mapping as in plot()
                level_to_color = {
                    'DEBUG': 'black',
                    'INFO': 'black',
                    'WARNING': 'orange',
                    'ERROR': 'red',
                    'CRITICAL': 'red',
                }
                # group visible events by color
                colors_groups = {}
                for t, lvl in zip(xs_vis, levels_vis):
                    c = level_to_color.get(lvl, 'k')
                    colors_groups.setdefault(c, []).append(t)
                ev_items = self.plot_items.get((i_panel, 'event'))
                if ev_items is None:
                    ev_items = {}
                    for color, t_list in colors_groups.items():
                        m = len(t_list)
                        xs = np.empty(3 * m)
                        ys = np.empty(3 * m)
                        xs[0::3] = t_list
                        xs[1::3] = t_list
                        xs[2::3] = np.nan
                        ys[0::3] = y_bottom
                        ys[1::3] = y_top
                        ys[2::3] = np.nan
                        item = pg.PlotDataItem(xs, ys, pen=pg.mkPen(color, width=1), name='event')
                        plot.addItem(item)
                        ev_items[color] = item
                    if len(ev_items) > 0:
                        self.plot_items[(i_panel, 'event')] = ev_items
                else:
                    # update existing items, create new ones if needed
                    # ev_items: dict color -> PlotDataItem
                    remaining = dict(colors_groups)
                    for color, item in list(ev_items.items()):
                        t_list = colors_groups.get(color, [])
                        if len(t_list) == 0:
                            try:
                                item.setData([], [])
                            except Exception:
                                pass
                        else:
                            m = len(t_list)
                            xs = np.empty(3 * m)
                            ys = np.empty(3 * m)
                            xs[0::3] = t_list
                            xs[1::3] = t_list
                            xs[2::3] = np.nan
                            ys[0::3] = y_bottom
                            ys[1::3] = y_top
                            ys[2::3] = np.nan
                            try:
                                item.setData(xs, ys)
                            except Exception:
                                try:
                                    item.setData(x=xs, y=ys)
                                except Exception:
                                    pass
                        if color in remaining:
                            del remaining[color]
                    # create items for any new color groups
                    for color, t_list in remaining.items():
                        m = len(t_list)
                        xs = np.empty(3 * m)
                        ys = np.empty(3 * m)
                        xs[0::3] = t_list
                        xs[1::3] = t_list
                        xs[2::3] = np.nan
                        ys[0::3] = y_bottom
                        ys[1::3] = y_top
                        ys[2::3] = np.nan
                        item = pg.PlotDataItem(xs, ys, pen=pg.mkPen(color, width=1), name='event')
                        plot.addItem(item)
                        ev_items[color] = item
                logger.info(f"_on_range_debounced: panel={i_panel} var={varname} events_drawn={sum(len(v.getData()[0])//3 for v in ev_items.values()) if ev_items is not None else 0}")
        logger.info(f"_on_range_debounced done elapsed={time.perf_counter()-t0_global:.3f}s")

    def reset_view(self):
        # スケールリセットボタン押下時、全パネルの表示範囲を初期化
        for plot in self.plots:
            plot.enableAutoRange(axis=pg.ViewBox.XYAxes)

    def on_hover(self, i, evt):
        # イベント有無パネル上でマウスを動かしたとき、該当時刻のイベント内容をポップアップ表示
        pos = evt[0]
        vb = self.plots[i].getViewBox()
        if vb.sceneBoundingRect().contains(pos):
            mouse_point = vb.mapSceneToView(pos)
            x = mouse_point.x()
            # 最近傍のイベントを探す
            if self.event_df is None or len(self.event_df) == 0:
                return
            idx = (abs(self.event_df['time'] - x)).argmin()
            row = self.event_df.iloc[idx]
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), row['original_message'])


    def _get_ts_y_for_var(self, df, varname):
        """Return (ts, y) numpy arrays for a requested varname from df.
        Supports plain column names (e.g. 'J1') and joint-specific derived names
        like 'J1_max_ratio' or 'J2_accel_max_ratio'. Returns (None, None) when
        the df doesn't contain the requested data.
        """
        if df is None:
            return None, None
        # timestamps
        if 'time' not in df.columns:
            return None, None
        ts = df['time'].values
        # direct column
        if varname in df.columns:
            return ts, df[varname].values
        # joint-specific composite e.g. J3_max_ratio
        m = re.match(r'^J([1-6])_(max_ratio|accel_max_ratio)$', varname)
        if m:
            joint_idx = int(m.group(1)) - 1
            base_field = m.group(2)
            # control message stores these under 'max_ratio' / 'accel_max_ratio'
            if base_field in df.columns:
                col = df[base_field]
                # if entries are list-like per-row, convert to 2D array
                try:
                    if col.dtype == object:
                        arr2d = np.asarray(col.to_list())
                    else:
                        arr2d = np.asarray(col)
                except Exception:
                    return ts, np.full(len(ts), np.nan)
                # arr2d could be (N,) scalar per row or (N, M)
                if arr2d.ndim == 1:
                    # scalar per row -> broadcast
                    return ts, arr2d.astype(float)
                elif arr2d.ndim == 2:
                    if arr2d.shape[1] > joint_idx:
                        return ts, arr2d[:, joint_idx].astype(float)
                    else:
                        return ts, np.full(len(ts), np.nan)
        return None, None


if __name__ == '__main__':
    from PyQt6.QtGui import QFont
    # 日本語対応フォント名（環境に応じて変更可）
    # 例: "IPAGothic", "Noto Sans CJK JP", "VL Gothic" など
    jp_font = QFont("IPAGothic")
    jp_font.setPointSize(10)
    QtWidgets.QApplication.setFont(jp_font)
    app = QtWidgets.QApplication(sys.argv)
    log_dir = 'log'
    win = LogViewer(log_dir)
    win.setWindowTitle('Log Viewer')
    win.show()
    sys.exit(app.exec())
