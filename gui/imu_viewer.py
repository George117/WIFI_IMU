"""
WiFi IMU Viewer — Real-time display of ESP8266 7-DOF IMU data.
Receives 43-byte UDP binary packets and plots accel, gyro, pressure,
and temperature using PyQtGraph.  See ../PROTOCOL.md for packet details.
"""

import argparse
import csv
import json
import socket
import struct
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

# ── Protocol constants ───────────────────────────────────────────────────────
PACKET_FORMAT = "<2s I I 3f 3f f f B"
PACKET_SIZE = 43
SYNC_WORD = b"\xaa\x55"

CALIBRATE_CMD = b"\xca\xfe"

DEFAULT_CONFIG = {
    "ip": "0.0.0.0",
    "port": 4269,
    "esp_ip": "",
    "axes": {
        "accel_x": "Accel X",
        "accel_y": "Accel Y",
        "accel_z": "Accel Z",
        "gyro_x": "Gyro X",
        "gyro_y": "Gyro Y",
        "gyro_z": "Gyro Z",
    },
}
CONFIG_PATH = Path(__file__).parent / "config.json"

AXIS_KEYS = ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z")
ALL_PLOT_KEYS = AXIS_KEYS + ("pressure", "temperature")

CSV_HEADER = [
    "timestamp_ms", "packet_id",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "pressure", "temperature",
]


def load_config() -> dict:
    config = json.loads(json.dumps(DEFAULT_CONFIG))  # deep copy
    config.setdefault("time_window", 10.0)
    config["visible"] = {k: True for k in ALL_PLOT_KEYS}
    config["colors"] = {}
    if CONFIG_PATH.exists():
        try:
            with open(CONFIG_PATH) as f:
                user = json.load(f)
            if "ip" in user:
                config["ip"] = str(user["ip"])
            if "port" in user:
                config["port"] = int(user["port"])
            if "esp_ip" in user:
                config["esp_ip"] = str(user["esp_ip"])
            if "time_window" in user:
                config["time_window"] = float(user["time_window"])
            if "axes" in user and isinstance(user["axes"], dict):
                for key in AXIS_KEYS:
                    if key in user["axes"]:
                        config["axes"][key] = str(user["axes"][key])
            if "visible" in user and isinstance(user["visible"], dict):
                for key in ALL_PLOT_KEYS:
                    if key in user["visible"]:
                        config["visible"][key] = bool(user["visible"][key])
            if "colors" in user and isinstance(user["colors"], dict):
                for key in ALL_PLOT_KEYS:
                    if key in user["colors"]:
                        config["colors"][key] = str(user["colors"][key])
        except (json.JSONDecodeError, ValueError, OSError) as e:
            print(f"Warning: could not load {CONFIG_PATH}: {e}")
    return config


def parse_packet(data: bytes) -> dict | None:
    if len(data) != PACKET_SIZE:
        return None
    if data[0:2] != SYNC_WORD:
        return None

    computed = 0
    for b in data[:42]:
        computed ^= b
    if computed != data[42]:
        return None

    fields = struct.unpack(PACKET_FORMAT, data)
    return {
        "packet_id": fields[1],
        "timestamp_ms": fields[2],
        "accel": (fields[3], fields[4], fields[5]),
        "gyro": (fields[6], fields[7], fields[8]),
        "pressure": fields[9],
        "temperature": fields[10],
    }


# ── Ring buffer ──────────────────────────────────────────────────────────────
class RingBuffer:
    """Fixed-capacity ring buffer backed by a 2-D NumPy array."""

    def __init__(self, capacity: int, channels: int):
        self.capacity = capacity
        self.channels = channels
        self.data = np.zeros((capacity, channels), dtype=np.float64)
        self.index = 0
        self.count = 0

    def append(self, row: np.ndarray):
        self.data[self.index] = row
        self.index = (self.index + 1) % self.capacity
        if self.count < self.capacity:
            self.count += 1

    def get(self) -> np.ndarray:
        """Return (count, channels) array in chronological order."""
        if self.count < self.capacity:
            return self.data[: self.count]
        return np.roll(self.data, -self.index, axis=0)


# ── UDP receiver thread ─────────────────────────────────────────────────────
class ReceiverThread(QtCore.QThread):
    packet_received = QtCore.Signal(dict)

    def __init__(self, bind_ip: str, port: int):
        super().__init__()
        self._bind_ip = bind_ip
        self._port = port
        self._running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._bind_ip, self._port))
        sock.settimeout(1.0)

        while self._running:
            try:
                data, _ = sock.recvfrom(256)
            except socket.timeout:
                continue
            pkt = parse_packet(data)
            if pkt is not None:
                self.packet_received.emit(pkt)

        sock.close()

    def stop(self):
        self._running = False
        self.wait()


# ── Main window ──────────────────────────────────────────────────────────────
PLOT_FPS = 30
SAMPLE_RATE = 100  # Hz

# Column indices in the ring buffer
COL_TIME = 0
COL_AX, COL_AY, COL_AZ = 1, 2, 3
COL_GX, COL_GY, COL_GZ = 4, 5, 6
COL_PRESS = 7
COL_TEMP = 8
NUM_CHANNELS = 9

# Plot key → (default_title, unit, column_index, color, y_range)
_ALL_PLOT_INFO = {
    "accel_x":     ("Accel X",     "g",     COL_AX,    "#e74c3c", (-4, 4)),
    "accel_y":     ("Accel Y",     "g",     COL_AY,    "#2ecc71", (-4, 4)),
    "accel_z":     ("Accel Z",     "g",     COL_AZ,    "#3498db", (-4, 4)),
    "gyro_x":      ("Gyro X",      "deg/s", COL_GX,    "#e74c3c", (-500, 500)),
    "gyro_y":      ("Gyro Y",      "deg/s", COL_GY,    "#2ecc71", (-500, 500)),
    "gyro_z":      ("Gyro Z",      "deg/s", COL_GZ,    "#3498db", (-500, 500)),
    "pressure":    ("Pressure",    "Pa",    COL_PRESS, "#f1c40f", None),
    "temperature": ("Temperature", "°C",    COL_TEMP,  "#1abc9c", None),
}


def build_plot_defs(axes: dict, visible: dict, colors: dict) -> list:
    """Build plot definitions list, filtered by visibility."""
    defs = []
    for key in ALL_PLOT_KEYS:
        if not visible.get(key, True):
            continue
        default_title, unit, col, default_color, yrange = _ALL_PLOT_INFO[key]
        title = axes.get(key, default_title)
        color = colors.get(key, default_color)
        defs.append((key, title, unit, col, color, yrange))
    return defs


class IMUViewer(QtWidgets.QMainWindow):
    def __init__(self, default_ip: str, default_port: int, esp_ip: str,
                 plot_defs: list, time_window: float = 10.0):
        super().__init__()
        self.setWindowTitle("WiFi IMU Viewer")
        self.resize(1000, 900)

        self._time_window = time_window
        buf_capacity = int(self._time_window * SAMPLE_RATE)
        self._buf = RingBuffer(buf_capacity, NUM_CHANNELS)
        self._t0: float | None = None
        self._last_id: int | None = None
        self._total_rx = 0
        self._drops = 0
        self._rate_counter = 0
        self._rate_value = 0.0
        self._receiver: ReceiverThread | None = None
        self._esp_ip = esp_ip
        self._paused = False
        self._fft_mode = False
        self._csv_file = None
        self._csv_writer = None
        self._plot_defs = plot_defs

        # ── UI ────────────────────────────────────────────────────────────
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        # ── Connection form ───────────────────────────────────────────────
        form = QtWidgets.QHBoxLayout()
        form.addWidget(QtWidgets.QLabel("Bind IP:"))
        self._ip_edit = QtWidgets.QLineEdit(default_ip)
        self._ip_edit.setFixedWidth(140)
        form.addWidget(self._ip_edit)

        form.addWidget(QtWidgets.QLabel("Port:"))
        self._port_edit = QtWidgets.QLineEdit(str(default_port))
        self._port_edit.setFixedWidth(70)
        form.addWidget(self._port_edit)

        self._start_btn = QtWidgets.QPushButton("Start")
        self._start_btn.setFixedWidth(80)
        self._start_btn.clicked.connect(self._toggle_receiver)
        form.addWidget(self._start_btn)

        # ── Pause button ─────────────────────────────────────────────────
        self._pause_btn = QtWidgets.QPushButton("Pause")
        self._pause_btn.setFixedWidth(80)
        self._pause_btn.setEnabled(False)
        self._pause_btn.clicked.connect(self._toggle_pause)
        form.addWidget(self._pause_btn)

        # ── Record button ────────────────────────────────────────────────
        self._rec_btn = QtWidgets.QPushButton("Record")
        self._rec_btn.setFixedWidth(80)
        self._rec_btn.setEnabled(False)
        self._rec_btn.clicked.connect(self._toggle_recording)
        form.addWidget(self._rec_btn)

        # ── FFT toggle button ────────────────────────────────────────────
        self._fft_btn = QtWidgets.QPushButton("FFT")
        self._fft_btn.setFixedWidth(80)
        self._fft_btn.setCheckable(True)
        self._fft_btn.clicked.connect(self._toggle_fft)
        form.addWidget(self._fft_btn)

        # ── Calibrate button ─────────────────────────────────────────────
        self._cal_btn = QtWidgets.QPushButton("Calibrate")
        self._cal_btn.setFixedWidth(80)
        self._cal_btn.setEnabled(False)
        self._cal_btn.clicked.connect(self._send_calibrate)
        form.addWidget(self._cal_btn)

        form.addStretch()

        self._status = QtWidgets.QLabel("Stopped")
        self._status.setStyleSheet("color: #aaa; font-family: monospace;")
        form.addWidget(self._status)

        root.addLayout(form)

        # ── Plot area ─────────────────────────────────────────────────────
        pg.setConfigOptions(antialias=False, useOpenGL=True)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        root.addWidget(scroll)

        num_cols = 3
        num_rows = (len(plot_defs) + num_cols - 1) // num_cols

        self._graphics = pg.GraphicsLayoutWidget()
        scroll.setWidget(self._graphics)
        self._graphics.setMinimumHeight(num_rows * 200)

        self._plots: list[pg.PlotItem] = []
        self._curves: list[pg.PlotDataItem] = []
        self._col_indices: list[int] = []
        self._stat_labels: list[pg.TextItem] = []

        for i, (_key, title, unit, col_idx, color, yrange) in enumerate(plot_defs):
            r, c = divmod(i, num_cols)
            p = self._graphics.addPlot(row=r, col=c)
            p.setLabel("left", unit)
            p.setLabel("bottom", "s")
            p.setTitle(title, size="9pt")
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setClipToView(True)
            p.setDownsampling(mode="peak")
            if yrange:
                p.setYRange(*yrange)
            curve = p.plot(pen=pg.mkPen(color, width=1))
            self._plots.append(p)
            self._curves.append(curve)
            self._col_indices.append(col_idx)

            # Statistics text overlay (top-left of each plot)
            stat = pg.TextItem("", anchor=(0, 0), color="#aaaaaa")
            stat.setFont(QtGui.QFont("Monospace", 8))
            stat.setParentItem(p.vb)
            self._stat_labels.append(stat)

        # Link all time-domain x-axes to the first plot
        for p in self._plots[1:]:
            p.setXLink(self._plots[0])

        # ── FFT plots (same grid, below time-domain) ─────────────────────
        fft_row_offset = num_rows
        self._fft_plots: list[pg.PlotItem] = []
        self._fft_curves: list[pg.PlotDataItem] = []

        for i, (_key, title, unit, col_idx, color, yrange) in enumerate(plot_defs):
            r, c = divmod(i, num_cols)
            p = self._graphics.addPlot(row=fft_row_offset + r, col=c)
            p.setLabel("left", unit)
            p.setLabel("bottom", "Hz")
            p.setTitle(f"{title} — FFT", size="9pt")
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setClipToView(True)
            p.setXRange(0, SAMPLE_RATE / 2, padding=0)
            curve = p.plot(pen=pg.mkPen(color, width=1))
            p.setVisible(False)
            self._fft_plots.append(p)
            self._fft_curves.append(curve)

        # Link all FFT x-axes
        for p in self._fft_plots[1:]:
            p.setXLink(self._fft_plots[0])

        fft_total_rows = (len(plot_defs) + num_cols - 1) // num_cols
        self._fft_row_count = fft_total_rows

        # ── Timers ────────────────────────────────────────────────────────
        self._plot_timer = QtCore.QTimer()
        self._plot_timer.timeout.connect(self._update_plots)

        self._rate_timer = QtCore.QTimer()
        self._rate_timer.timeout.connect(self._update_rate)

    # ── FFT control ──────────────────────────────────────────────────────
    def _toggle_fft(self):
        self._fft_mode = self._fft_btn.isChecked()
        for p in self._fft_plots:
            p.setVisible(self._fft_mode)
        # Resize graphics widget to fit
        num_cols = 3
        time_rows = (len(self._plot_defs) + num_cols - 1) // num_cols
        total_rows = time_rows + (self._fft_row_count if self._fft_mode else 0)
        self._graphics.setMinimumHeight(total_rows * 200)

    # ── Calibration ────────────────────────────────────────────────────
    def _send_calibrate(self):
        if not self._esp_ip:
            self._status.setText("esp_ip not set in config.json")
            return
        try:
            port = int(self._port_edit.text().strip())
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(CALIBRATE_CMD, (self._esp_ip, port))
            sock.close()
            self._cal_btn.setText("Calibrating...")
            self._cal_btn.setStyleSheet("color: #f1c40f; font-weight: bold;")
            self._cal_btn.setEnabled(False)
            self._status.setText("Calibrating — keep sensor still...")
            QtCore.QTimer.singleShot(3000, self._cal_done)
        except OSError as e:
            self._status.setText(f"Calibration send failed: {e}")

    def _cal_done(self):
        self._cal_btn.setText("Calibrate")
        self._cal_btn.setStyleSheet("")
        self._cal_btn.setEnabled(True)
        self._status.setText("Calibration complete")

    # ── Pause control ────────────────────────────────────────────────────
    def _toggle_pause(self):
        self._paused = not self._paused
        self._pause_btn.setText("Resume" if self._paused else "Pause")

    # ── CSV recording ────────────────────────────────────────────────────
    def _toggle_recording(self):
        if self._csv_file is not None:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(__file__).parent / f"recording_{ts}.csv"
        self._csv_file = open(path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CSV_HEADER)
        self._rec_btn.setText("Stop Rec")
        self._rec_btn.setStyleSheet("color: #e74c3c; font-weight: bold;")
        self._status.setText(f"Recording → {path.name}")

    def _stop_recording(self):
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self._rec_btn.setText("Record")
        self._rec_btn.setStyleSheet("")

    # ── Connection control ────────────────────────────────────────────────
    def _toggle_receiver(self):
        if self._receiver is not None:
            self._stop_receiver()
        else:
            self._start_receiver()

    def _start_receiver(self):
        bind_ip = self._ip_edit.text().strip()
        try:
            port = int(self._port_edit.text().strip())
        except ValueError:
            self._status.setText("Invalid port")
            return

        # Reset state
        self._buf = RingBuffer(int(self._time_window * SAMPLE_RATE), NUM_CHANNELS)
        self._t0 = None
        self._last_id = None
        self._total_rx = 0
        self._drops = 0
        self._rate_counter = 0
        self._rate_value = 0.0
        self._paused = False
        self._pause_btn.setText("Pause")

        self._receiver = ReceiverThread(bind_ip, port)
        self._receiver.packet_received.connect(self._on_packet)
        self._receiver.start()

        self._plot_timer.start(1000 // PLOT_FPS)
        self._rate_timer.start(1000)

        self._ip_edit.setEnabled(False)
        self._port_edit.setEnabled(False)
        self._start_btn.setText("Stop")
        self._pause_btn.setEnabled(True)
        self._rec_btn.setEnabled(True)
        self._cal_btn.setEnabled(bool(self._esp_ip))
        self._status.setText("Waiting for data…")

    def _stop_receiver(self):
        if self._receiver is not None:
            self._receiver.stop()
            self._receiver = None

        self._stop_recording()
        self._plot_timer.stop()
        self._rate_timer.stop()

        self._ip_edit.setEnabled(True)
        self._port_edit.setEnabled(True)
        self._start_btn.setText("Start")
        self._pause_btn.setEnabled(False)
        self._rec_btn.setEnabled(False)
        self._cal_btn.setEnabled(False)
        self._status.setText("Stopped")

    # ── Slots ─────────────────────────────────────────────────────────────
    def _on_packet(self, pkt: dict):
        ts = pkt["timestamp_ms"]

        if self._t0 is not None and ts < (self._t0 - 5000):
            self._t0 = None

        if self._t0 is None:
            self._t0 = ts

        t = (ts - self._t0) / 1000.0

        row = np.array([
            t,
            *pkt["accel"],
            *pkt["gyro"],
            pkt["pressure"],
            pkt["temperature"],
        ])
        self._buf.append(row)

        # CSV recording
        if self._csv_writer is not None:
            self._csv_writer.writerow([
                pkt["timestamp_ms"], pkt["packet_id"],
                *pkt["accel"], *pkt["gyro"],
                pkt["pressure"], pkt["temperature"],
            ])

        pid = pkt["packet_id"]
        if self._last_id is not None and pid > self._last_id + 1:
            self._drops += pid - self._last_id - 1
        self._last_id = pid
        self._total_rx += 1
        self._rate_counter += 1

    def _update_rate(self):
        self._rate_value = self._rate_counter
        self._rate_counter = 0

    def _update_plots(self):
        if self._paused:
            return

        d = self._buf.get()
        if len(d) == 0:
            return

        t = d[:, COL_TIME]

        for i, (curve, col_idx) in enumerate(zip(self._curves, self._col_indices)):
            y = d[:, col_idx]
            curve.setData(t, y)

            # Update statistics overlay
            if len(y) > 0:
                vmin = np.min(y)
                vmax = np.max(y)
                vmean = np.mean(y)
                vrms = np.sqrt(np.mean(y ** 2))
                self._stat_labels[i].setText(
                    f"Min:{vmin:8.2f}  Max:{vmax:8.2f}\n"
                    f"Avg:{vmean:8.2f}  RMS:{vrms:8.2f}"
                )
                # Position at top-left of visible area
                vb = self._plots[i].vb
                rect = vb.viewRect()
                self._stat_labels[i].setPos(rect.left(), rect.top())

        # ── FFT update ────────────────────────────────────────────────
        if self._fft_mode and len(d) >= 4:
            n = len(d)
            freqs = np.fft.rfftfreq(n, d=1.0 / SAMPLE_RATE)
            for i, col_idx in enumerate(self._col_indices):
                y = d[:, col_idx]
                y_detrend = y - np.mean(y)
                fft_mag = np.abs(np.fft.rfft(y_detrend)) * 2.0 / n
                self._fft_curves[i].setData(freqs, fft_mag)

        t_max = t[-1]
        self._plots[0].setXRange(t_max - self._time_window, t_max, padding=0)

        rec_str = " | REC" if self._csv_file is not None else ""
        pause_str = " | PAUSED" if self._paused else ""
        self._status.setText(
            f"Rate: {self._rate_value:.0f} Hz  |  "
            f"Received: {self._total_rx}  |  "
            f"Dropped: {self._drops}"
            f"{rec_str}{pause_str}"
        )

    def closeEvent(self, event):
        self._stop_receiver()
        event.accept()


# ── Entry point ──────────────────────────────────────────────────────────────
def main():
    config = load_config()

    parser = argparse.ArgumentParser(description="WiFi IMU Viewer")
    parser.add_argument("--ip", default=None, help="Bind IP address")
    parser.add_argument("--port", type=int, default=None, help="UDP listen port")
    args = parser.parse_args()

    ip = args.ip if args.ip is not None else config["ip"]
    port = args.port if args.port is not None else config["port"]

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = app.palette()
    palette.setColor(palette.ColorRole.Window, pg.mkColor("#1e1e1e"))
    palette.setColor(palette.ColorRole.WindowText, pg.mkColor("#cccccc"))
    palette.setColor(palette.ColorRole.Base, pg.mkColor("#1e1e1e"))
    app.setPalette(palette)
    pg.setConfigOption("background", "#1e1e1e")
    pg.setConfigOption("foreground", "#cccccc")

    plot_defs = build_plot_defs(config["axes"], config["visible"], config["colors"])
    window = IMUViewer(ip, port, config["esp_ip"], plot_defs, config["time_window"])
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
