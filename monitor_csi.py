#!/usr/bin/env python3
"""
Real-time CSI visualization from ESP32 RX via Serial.

Main view:
    - CSI brute: raw I/Q magnitude, shown as subcarriers x received frames.
    - CSI corrigee: same CSI after gain compensation in dB.

Optional views are opened only when clicking the buttons in the main figure.
"""

import argparse
import re
import struct
import sys
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


class CSIMonitor:
    def __init__(self, port='COM23', baudrate=2000000, buffer_size=100, csi_history_size=800, plot_window=5.0):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        self.csi_history_size = csi_history_size
        self.plot_window = plot_window

        self.timestamps = deque(maxlen=buffer_size)
        self.rssi_values = deque(maxlen=buffer_size)
        self.agc_gains = deque(maxlen=buffer_size)
        self.fft_gains = deque(maxlen=buffer_size)
        self.compensation_gains = deque(maxlen=buffer_size)

        self.csi_raw = deque(maxlen=buffer_size)
        self.csi_corrected = deque(maxlen=buffer_size)
        self.csi_raw_db = deque(maxlen=buffer_size)
        self.csi_corrected_db = deque(maxlen=buffer_size)

        self.csi_raw_db_history = deque(maxlen=csi_history_size)
        self.csi_corr_db_history = deque(maxlen=csi_history_size)
        self.csi_time_history = deque(maxlen=csi_history_size)

        self.frame_count = 0
        self.start_time = datetime.now()
        self.first_device_time_us = None
        self.last_device_time_us = None
        self.rate_host_times = deque(maxlen=100)
        self.rate_device_times = deque(maxlen=100)
        self.last_meta = None
        self.last_csi_len = 0
        self.last_applied_comp = 0
        self.unparsed_csi_count = 0
        self.serial_buffer = bytearray()
        self.binary_header = struct.Struct('<4sBHQb6s')
        self.binary_mode = False

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            print(f"Connected to {port} at {baudrate} baud")
        except Exception as exc:
            print(f"Failed to open {port}: {exc}")
            sys.exit(1)

        self.csi_pattern = re.compile(
            r'CSI: len=(\d+) mac=([0-9a-fA-F:]+) rssi=(-?\d+) agc=(\d+) fft=(-?\d+) comp=(-?\d+)'
        )
        self.csi_bytes_pattern = re.compile(r'CSI_BYTES:(.+)')

    def parse_csi_line(self, line):
        match = self.csi_pattern.search(line)
        if not match:
            return None

        return {
            'len': int(match.group(1)),
            'mac': match.group(2),
            'rssi': int(match.group(3)),
            'agc': int(match.group(4)),
            'fft': int(match.group(5)),
            'comp': int(match.group(6)),
        }

    def parse_csi_bytes(self, line):
        match = self.csi_bytes_pattern.search(line)
        if not match:
            return None

        return self.parse_sample_tokens(match.group(1).strip().split())

    def parse_csi_meta(self, line):
        if not line.startswith('CSI_META,'):
            return None

        parts = line.split(',')
        if len(parts) < 16:
            return None

        sample_field = parts[-1].strip().split()
        if len(sample_field) < 2:
            return None

        try:
            csi_len = int(sample_field[0], 0)
            rssi = int(parts[10], 0) if parts[10] else 0
        except ValueError:
            return None

        samples = self.parse_sample_tokens(sample_field[1:])
        if not samples:
            return None

        return {
            # CSI_META carries the raw CSI samples. The gain fields are not in
            # this line, so keep using the latest preceding "CSI:" summary.
            'meta': {
                'len': csi_len,
                'mac': parts[2],
                'rssi': rssi,
                'agc': 0,
                'fft': 0,
                'comp': 0,
            },
            'samples': samples,
        }

    @staticmethod
    def parse_sample_tokens(tokens):
        samples = []
        for token in tokens:
            token = token.strip()
            if not token:
                continue

            try:
                value = int(token, 0)
            except ValueError:
                try:
                    value = int(token, 16)
                except ValueError:
                    return None

            samples.append(value & 0xff)

        return samples

    @staticmethod
    def bytes_to_iq_magnitude(csi_bytes):
        """Convert ESP32 signed I/Q bytes to one magnitude per subcarrier."""
        if len(csi_bytes) < 2:
            return np.array([], dtype=np.float32)

        values = np.array(csi_bytes, dtype=np.uint8).astype(np.int16)
        values[values >= 128] -= 256

        if len(values) % 2:
            values = values[:-1]

        i_values = values[0::2].astype(np.float32)
        q_values = values[1::2].astype(np.float32)
        return np.hypot(i_values, q_values)

    @staticmethod
    def to_db(magnitudes):
        return 20.0 * np.log10(np.maximum(magnitudes, 1e-6))

    @staticmethod
    def robust_limits(data, padding=2.0):
        if data.size == 0:
            return 0.0, 1.0

        vmin, vmax = np.nanpercentile(data, [2, 98])
        if not np.isfinite(vmin) or not np.isfinite(vmax) or vmin == vmax:
            vmin = float(np.nanmin(data))
            vmax = float(np.nanmax(data))
        if vmin == vmax:
            vmax = vmin + 1.0
        return vmin - padding, vmax + padding

    def read_serial(self):
        try:
            binary_frame = self.read_binary_frame()
            if binary_frame:
                self.record_meta(binary_frame['meta'])
                self.record_csi_samples(binary_frame['samples'])
                return binary_frame

            if not self.ser.in_waiting:
                return None

            data = self.ser.read(self.ser.in_waiting)
            if not data:
                return None

            self.serial_buffer.extend(data)

            binary_frame = self.read_binary_frame()
            if binary_frame:
                self.binary_mode = True
                self.record_meta(binary_frame['meta'])
                self.record_csi_samples(binary_frame['samples'])
                return binary_frame

            if self.binary_mode or b'CSIB' in self.serial_buffer:
                self.binary_mode = True
                return None

            line = self.read_text_line()
            if not line:
                return None

            csi_meta = self.parse_csi_meta(line)
            if csi_meta:
                self.record_meta(csi_meta['meta'])
                self.record_csi_samples(csi_meta['samples'])
                return csi_meta

            csi_data = self.parse_csi_line(line)
            if csi_data:
                self.record_meta(csi_data)
                return csi_data

            csi_bytes = self.parse_csi_bytes(line)
            if not csi_bytes:
                if 'CSI' in line:
                    self.unparsed_csi_count += 1
                    if self.unparsed_csi_count <= 5:
                        print(f"Unparsed CSI line #{self.unparsed_csi_count}: {line[:240]}")
                return None

            self.record_csi_samples(csi_bytes)

        except Exception as exc:
            print(f"Error reading serial: {exc}")

        return None

    def read_binary_frame(self):
        magic = b'CSIB'

        while True:
            start = self.serial_buffer.find(magic)
            if start < 0:
                if self.binary_mode and len(self.serial_buffer) > 3:
                    del self.serial_buffer[:-3]
                return None

            if start > 0:
                del self.serial_buffer[:start]

            if len(self.serial_buffer) < self.binary_header.size:
                return None

            header = self.serial_buffer[:self.binary_header.size]
            magic_value, version, csi_len, time_us, rssi, mac_bytes = self.binary_header.unpack(header)
            if magic_value != magic or version != 1 or csi_len > 512:
                del self.serial_buffer[0]
                continue

            frame_len = self.binary_header.size + csi_len
            if len(self.serial_buffer) < frame_len:
                return None

            samples = list(self.serial_buffer[self.binary_header.size:frame_len])
            del self.serial_buffer[:frame_len]

            mac = ':'.join(f'{byte:02x}' for byte in mac_bytes)
            return {
                'meta': {
                    'len': csi_len,
                    'mac': mac,
                    'rssi': rssi,
                    'agc': 0,
                    'fft': 0,
                    'comp': 0,
                    'time_us': time_us,
                },
                'samples': samples,
            }

    def read_text_line(self):
        newline = self.serial_buffer.find(b'\n')
        if newline < 0:
            if len(self.serial_buffer) > 4096:
                del self.serial_buffer[:-3]
            return None

        raw_line = bytes(self.serial_buffer[:newline + 1])
        del self.serial_buffer[:newline + 1]
        return raw_line.decode('utf-8', errors='ignore').strip()

    def record_meta(self, csi_data):
        self.frame_count += 1
        elapsed = (datetime.now() - self.start_time).total_seconds()

        self.timestamps.append(elapsed)
        self.rate_host_times.append(elapsed)
        self.rssi_values.append(csi_data['rssi'])
        self.agc_gains.append(csi_data['agc'])
        self.fft_gains.append(csi_data['fft'])
        self.compensation_gains.append(csi_data['comp'])
        self.last_meta = csi_data

        if self.frame_count <= 3 or self.frame_count % 100 == 0:
            rate = None
            if len(self.rate_host_times) > 1:
                host_span = self.rate_host_times[-1] - self.rate_host_times[0]
                if host_span > 0:
                    rate = (len(self.rate_host_times) - 1) / host_span

            device_rate = None
            time_us = csi_data.get('time_us')
            if time_us is not None:
                if self.first_device_time_us is None:
                    self.first_device_time_us = time_us
                self.last_device_time_us = time_us
                self.rate_device_times.append(time_us / 1_000_000.0)
                if len(self.rate_device_times) > 1:
                    device_span = self.rate_device_times[-1] - self.rate_device_times[0]
                    if device_span > 0:
                        device_rate = (len(self.rate_device_times) - 1) / device_span

            rate_text = f"{rate:6.1f}" if rate is not None else "   n/a"
            device_text = f" dev:{device_rate:6.1f} Hz" if device_rate is not None else ""
            print(
                f"[{self.frame_count:5d}] py:{rate_text} Hz{device_text} "
                f"RSSI:{csi_data['rssi']:4d} MAC:{csi_data['mac']}"
            )

    def record_csi_samples(self, csi_bytes):
        raw_subcarriers = self.bytes_to_iq_magnitude(csi_bytes)
        if raw_subcarriers.size == 0:
            return

        raw_db = self.to_db(raw_subcarriers)

        # Compensation is handled in dB. If comp is positive, the receiver
        # amplified the signal, so remove that gain from the displayed CSI.
        comp = self.compensation_gains[-1] if self.compensation_gains else 0
        self.last_applied_comp = comp
        corrected_db = raw_db - comp
        corrected_subcarriers = 10 ** (corrected_db / 20.0)

        self.csi_raw.append(raw_subcarriers)
        self.csi_corrected.append(corrected_subcarriers)
        self.csi_raw_db.append(raw_db)
        self.csi_corrected_db.append(corrected_db)
        self.csi_raw_db_history.append(raw_db)
        self.csi_corr_db_history.append(corrected_db)
        self.csi_time_history.append(self.timestamps[-1] if self.timestamps else 0.0)
        self.last_csi_len = len(csi_bytes)

        if self.frame_count <= 3 or self.frame_count % 25 == 0:
            print(
                f"     CSI bytes:{len(csi_bytes):4d} subcarriers:{raw_subcarriers.size:3d} "
                f"raw:{raw_db.min():6.1f}..{raw_db.max():6.1f} dB "
                f"corr:{corrected_db.min():6.1f}..{corrected_db.max():6.1f} dB "
                f"applied_comp:{comp:5d} dB"
            )

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")


def subcarrier_extent(n_subcarriers, n_frames):
    half = n_subcarriers // 2
    ymin = -half
    ymax = half - 1 if n_subcarriers % 2 == 0 else half
    return [0, max(1, n_frames), ymin, ymax]


def summarize_subcarriers(n_subcarriers):
    if n_subcarriers <= 0:
        return "aucun"

    half = n_subcarriers // 2
    first = -half
    last = half - 1 if n_subcarriers % 2 == 0 else half

    if n_subcarriers <= 16:
        values = [str(index) for index in range(first, last + 1)]
        return f"{', '.join(values)} ({n_subcarriers})"

    return f"{first}..{last} ({n_subcarriers})"


def style_heatmap_axis(ax, title):
    ax.set_title(title, fontsize=15, fontweight='bold', pad=10)
    ax.set_xlabel('Frames received', fontsize=12)
    ax.set_ylabel('Subcarrier', fontsize=12)
    ax.axhline(0, color='#6e2aa8', linewidth=1.0, alpha=0.8)
    ax.grid(False)


def history_to_matrix(history):
    frames = list(history)
    if not frames:
        return np.zeros((1, 1), dtype=np.float32)

    max_len = max(len(frame) for frame in frames)
    matrix = np.full((len(frames), max_len), np.nan, dtype=np.float32)
    for index, frame in enumerate(frames):
        matrix[index, :len(frame)] = frame
    return matrix.T


def windowed_histories(monitor):
    times = list(monitor.csi_time_history)
    raw_frames = list(monitor.csi_raw_db_history)
    corr_frames = list(monitor.csi_corr_db_history)
    if not times or not raw_frames or not corr_frames:
        return [], [], []

    latest = times[-1]
    start_time = max(0.0, latest - monitor.plot_window)
    selected = [
        (time_value, raw_frame, corr_frame)
        for time_value, raw_frame, corr_frame in zip(times, raw_frames, corr_frames)
        if time_value >= start_time
    ]
    if not selected:
        selected = [(times[-1], raw_frames[-1], corr_frames[-1])]

    win_times, win_raw, win_corr = zip(*selected)
    x = [time_value - win_times[-1] for time_value in win_times]
    return list(x), list(win_raw), list(win_corr)


def create_visualization(monitor):
    fig, (ax_raw, ax_corr) = plt.subplots(1, 2, figsize=(14, 7), sharey=True)
    fig.subplots_adjust(left=0.08, right=0.96, top=0.86, bottom=0.18, wspace=0.22)

    n_display = 64
    raw_lines = []
    corr_lines = []
    cmap_raw = plt.cm.viridis
    cmap_corr = plt.cm.plasma
    for i in range(n_display):
        color_r = cmap_raw(i / max(1, n_display - 1))
        color_c = cmap_corr(i / max(1, n_display - 1))
        (lr,) = ax_raw.plot([], [], color=color_r, linewidth=0.9, alpha=0.9)
        (lc,) = ax_corr.plot([], [], color=color_c, linewidth=0.9, alpha=0.9)
        raw_lines.append(lr)
        corr_lines.append(lc)

    ax_raw.set_title('CSI brute (sous-porteuses vs temps)', fontsize=14, fontweight='bold')
    ax_raw.set_xlabel('Temps (s)')
    ax_raw.set_ylabel('CSI amplitude (dB rel.)')
    ax_corr.set_title('CSI corrigee (sous-porteuses vs temps)', fontsize=14, fontweight='bold')
    ax_corr.set_xlabel('Temps (s)')
    ax_corr.set_ylabel('')

    status_text = fig.text(0.08, 0.92, 'Waiting for CSI...', fontsize=11)
    subcarrier_text = fig.text(0.08, 0.885, 'Subcarriers affiches: attente CSI...', fontsize=10)
    warning_text = fig.text(0.08, 0.08, '', fontsize=10, color='#9a3412')
    fig.suptitle('ESP32 CSI - brute vs corrigee', fontsize=17, fontweight='bold')

    detail_state = {'fig': None}
    frame_state = {'fig': None}

    def update_detail_fig():
        detail_fig = detail_state.get('fig')
        if detail_fig is None or not plt.fignum_exists(detail_fig.number):
            return

        ax_rssi, ax_agc, ax_comp = detail_fig.axes[:3]
        for ax in (ax_rssi, ax_agc, ax_comp):
            ax.clear()
            ax.grid(True, alpha=0.25)

        if monitor.timestamps:
            x_data = list(monitor.timestamps)
            x_min = max(0, x_data[-1] - 30)
            x_max = x_data[-1] + 1

            ax_rssi.plot(x_data, list(monitor.rssi_values), color='#2563eb', linewidth=1.5)
            ax_rssi.set_title('RSSI')
            ax_rssi.set_ylabel('dBm')
            ax_rssi.set_ylim(-100, 0)
            ax_rssi.set_xlim(x_min, x_max)

            ax_agc.plot(x_data, list(monitor.agc_gains), color='#16a34a', linewidth=1.5)
            ax_agc.set_title('AGC')
            ax_agc.set_ylabel('gain')
            ax_agc.set_xlim(x_min, x_max)

            ax_comp.plot(x_data, list(monitor.compensation_gains), color='#9333ea', linewidth=1.5)
            ax_comp.set_title('Compensation')
            ax_comp.set_ylabel('dB approx.')
            ax_comp.set_xlabel('Time (s)')
            ax_comp.set_xlim(x_min, x_max)

        detail_fig.canvas.draw_idle()

    def update_frame_fig():
        frame_fig = frame_state.get('fig')
        if frame_fig is None or not plt.fignum_exists(frame_fig.number):
            return

        ax_raw_line, ax_corr_line, ax_delta = frame_fig.axes[:3]
        for ax in (ax_raw_line, ax_corr_line, ax_delta):
            ax.clear()
            ax.grid(True, alpha=0.25)

        if monitor.csi_raw_db and monitor.csi_corrected_db:
            raw = monitor.csi_raw_db[-1]
            corr = monitor.csi_corrected_db[-1]
            x = np.arange(len(raw)) - len(raw) // 2

            ax_raw_line.plot(x, raw, color='#0f766e', linewidth=1.1)
            ax_raw_line.set_title('Derniere CSI brute')
            ax_raw_line.set_ylabel('CSI amplitude (dB rel.)')

            ax_corr_line.plot(x, corr, color='#7c3aed', linewidth=1.1)
            ax_corr_line.set_title('Derniere CSI corrigee')
            ax_corr_line.set_ylabel('CSI amplitude (dB rel.)')

            ax_delta.plot(x, corr - raw, color='#ea580c', linewidth=1.1)
            ax_delta.set_title('Correction appliquee')
            ax_delta.set_ylabel('Delta (dB)')
            ax_delta.set_xlabel('Subcarrier')

        frame_fig.canvas.draw_idle()

    def show_details(_event):
        if detail_state.get('fig') is None or not plt.fignum_exists(detail_state['fig'].number):
            detail_state['fig'], _axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
            detail_state['fig'].suptitle('Metriques radio', fontweight='bold')
            detail_state['fig'].subplots_adjust(hspace=0.45)
        update_detail_fig()
        detail_state['fig'].show()

    def show_frames(_event):
        if frame_state.get('fig') is None or not plt.fignum_exists(frame_state['fig'].number):
            frame_state['fig'], _axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
            frame_state['fig'].suptitle('Derniere trame CSI', fontweight='bold')
            frame_state['fig'].subplots_adjust(hspace=0.45)
        update_frame_fig()
        frame_state['fig'].show()

    ax_button_details = fig.add_axes([0.66, 0.035, 0.14, 0.055])
    ax_button_frames = fig.add_axes([0.82, 0.035, 0.14, 0.055])
    button_details = Button(ax_button_details, 'Metriques')
    button_frames = Button(ax_button_frames, 'Trame')
    button_details.on_clicked(show_details)
    button_frames.on_clicked(show_frames)

    def animate(_frame):
        for _ in range(50):
            had_data = monitor.ser.in_waiting > 0 or bool(monitor.serial_buffer)
            monitor.read_serial()
            if not had_data:
                break
        x_window, raw_history, corr_history = windowed_histories(monitor)
        if len(raw_history) > 1 and len(corr_history) > 1:
            raw = history_to_matrix(raw_history)
            corr = history_to_matrix(corr_history)
            frames = raw.shape[1]
            subcarriers = raw.shape[0]

            n_plot = min(n_display, subcarriers)
            if subcarriers > n_plot:
                start = max(0, subcarriers // 2 - n_plot // 2)
            else:
                start = 0
            indices = list(range(start, start + n_plot))

            x = np.array(x_window, dtype=np.float32)
            for line_idx, sc_idx in enumerate(indices):
                y_raw = raw[sc_idx, :]
                y_corr = corr[sc_idx, :]
                raw_lines[line_idx].set_data(x, y_raw)
                corr_lines[line_idx].set_data(x, y_corr)

            combined = np.concatenate([raw.ravel(), corr.ravel()])
            vmin, vmax = monitor.robust_limits(combined)
            for ax in (ax_raw, ax_corr):
                ax.set_xlim(-monitor.plot_window, 0)
                ax.set_ylim(vmin, vmax)

            if monitor.last_meta:
                status_text.set_text(
                    f"Frames: {monitor.frame_count} | CSI bytes: {monitor.last_csi_len} | "
                    f"Fenetre: {monitor.plot_window:.1f}s | Subcarriers: {subcarriers} | RSSI: {monitor.last_meta['rssi']} dBm | "
                    f"AGC: {monitor.last_meta['agc']} | COMP: {monitor.last_meta['comp']} | "
                    f"applied: {monitor.last_applied_comp} dB"
                )
                subcarrier_text.set_text(
                    f"Subcarriers affiches: {summarize_subcarriers(subcarriers)} (affichage: {n_plot})"
                )

            if monitor.last_csi_len < 128:
                warning_text.set_text(
                    "Attention: peu d'octets CSI recus. Flashe le RX modifie pour imprimer tout csi_info->len."
                )
            else:
                warning_text.set_text('')

        update_detail_fig()
        update_frame_fig()
        # Return all line artists and status texts (imshow artists removed)
        return tuple(raw_lines) + tuple(corr_lines) + (status_text, subcarrier_text, warning_text)

    ani = FuncAnimation(fig, animate, interval=100, blit=False, cache_frame_data=False)
    fig._csi_animation = ani
    fig._csi_buttons = (button_details, button_frames)

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='CSI Monitor - real-time visualization from ESP32')
    parser.add_argument('-p', '--port', default='COM23', help='Serial port (default: COM23)')
    parser.add_argument('-b', '--baud', type=int, default=2000000, help='Baud rate (default: 2000000)')
    parser.add_argument('-s', '--size', type=int, default=100, help='Metric buffer size (default: 100)')
    parser.add_argument('--history', type=int, default=800, help='CSI frames shown in heatmap (default: 800)')
    parser.add_argument('--window', type=float, default=5.0, help='CSI plot time window in seconds (default: 5.0)')
    args = parser.parse_args()

    monitor = CSIMonitor(
        port=args.port,
        baudrate=args.baud,
        buffer_size=args.size,
        csi_history_size=args.history,
        plot_window=args.window,
    )

    print("\n" + "=" * 60)
    print("ESP32 CSI Real-Time Monitor")
    print("=" * 60)
    print(f"Port: {args.port}, Baudrate: {args.baud}")
    print(f"CSI plot window: {args.window:.1f} seconds")
    print("Main window shows only CSI raw/corrected heatmaps.")
    print("Use the Metriques and Trame buttons for optional plots.")
    print("Close the plot window to exit.")
    print("=" * 60 + "\n")

    try:
        create_visualization(monitor)
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user")
    finally:
        monitor.close()


if __name__ == '__main__':
    main()
