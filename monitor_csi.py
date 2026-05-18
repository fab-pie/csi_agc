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
    def __init__(self, port='COM23', baudrate=2000000, buffer_size=100, csi_history_size=5000, plot_window=5.0, mac_filter=None):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        self.csi_history_size = csi_history_size
        self.plot_window = plot_window
        self.mac_filter = {mac.lower() for mac in (mac_filter or [])}

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
        self.reference_rssi = None
        self.unparsed_csi_count = 0
        self.serial_buffer = bytearray()
        self.binary_header_v1 = struct.Struct('<4sBHQb6s')
        self.binary_extra_v2 = struct.Struct('<Bbh')
        self.binary_mode = False
        self.last_accepted_time_us = None
        self.last_accepted_host_time = None
        self.filtered_count = 0

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

    @staticmethod
    def fnv1a32(data):
        value = 2166136261
        for byte in data:
            value ^= byte
            value = (value * 16777619) & 0xffffffff
        return value

    def accept_frame(self, frame):
        if self.mac_filter and frame['meta']['mac'].lower() not in self.mac_filter:
            self.filtered_count += 1
            return None

        self.record_meta(frame['meta'])
        self.record_csi_samples(frame['samples'])
        return frame

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
        db = np.full(magnitudes.shape, np.nan, dtype=np.float32)
        valid = magnitudes > 0
        db[valid] = 20.0 * np.log10(magnitudes[valid])
        return db

    @staticmethod
    def robust_limits(data, padding=2.0):
        finite = data[np.isfinite(data)]
        if finite.size == 0:
            return 0.0, 1.0

        vmin, vmax = np.percentile(finite, [2, 98])
        if not np.isfinite(vmin) or not np.isfinite(vmax) or vmin == vmax:
            vmin = float(np.min(finite))
            vmax = float(np.max(finite))
        if vmin == vmax:
            vmax = vmin + 1.0
        return vmin - padding, vmax + padding

    def read_serial(self):
        try:
            binary_frame = self.read_binary_frame()
            if binary_frame:
                return self.accept_frame(binary_frame)

            if not self.ser.in_waiting:
                return None

            data = self.ser.read(self.ser.in_waiting)
            if not data:
                return None

            self.serial_buffer.extend(data)

            binary_frame = self.read_binary_frame()
            if binary_frame:
                self.binary_mode = True
                return self.accept_frame(binary_frame)

            if self.binary_mode or b'CSIB' in self.serial_buffer:
                self.binary_mode = True
                return None

            line = self.read_text_line()
            if not line:
                return None

            csi_meta = self.parse_csi_meta(line)
            if csi_meta:
                return self.accept_frame(csi_meta)

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

            if len(self.serial_buffer) < self.binary_header_v1.size:
                return None

            header = self.serial_buffer[:self.binary_header_v1.size]
            magic_value, version, csi_len, time_us, rssi, mac_bytes = self.binary_header_v1.unpack(header)
            if magic_value != magic or version != 3 or csi_len > 512:
                del self.serial_buffer[0]
                continue

            extra_len = self.binary_extra_v2.size
            header_len = self.binary_header_v1.size + extra_len
            checksum_len = 4
            frame_len = header_len + csi_len + checksum_len
            if len(self.serial_buffer) < frame_len:
                return None

            agc = 0
            fft = 0
            comp = 0
            extra_start = self.binary_header_v1.size
            extra_end = extra_start + self.binary_extra_v2.size
            agc, fft, comp = self.binary_extra_v2.unpack(self.serial_buffer[extra_start:extra_end])

            payload_end = header_len + csi_len
            expected_checksum = struct.unpack('<I', self.serial_buffer[payload_end:frame_len])[0]
            actual_checksum = self.fnv1a32(self.serial_buffer[:payload_end])
            if expected_checksum != actual_checksum:
                del self.serial_buffer[0]
                continue

            samples = list(self.serial_buffer[header_len:payload_end])

            # CSI payload is arbitrary bytes, so the magic word can appear inside
            # samples. Reject implausible headers and keep scanning for the next
            # real frame instead of letting bogus timestamps poison device rate.
            plausible_len = csi_len in (128, 256)
            plausible_rssi = -100 <= rssi <= 0
            plausible_time = time_us > 0
            if self.last_accepted_time_us is not None:
                dt_us = time_us - self.last_accepted_time_us
                host_gap = (datetime.now() - self.last_accepted_host_time).total_seconds()
                plausible_time = dt_us >= 0 and (dt_us <= 2_000_000 or host_gap > 2.0)

            if not (plausible_len and plausible_rssi and plausible_time):
                del self.serial_buffer[0]
                continue

            del self.serial_buffer[:frame_len]
            self.last_accepted_time_us = time_us
            self.last_accepted_host_time = datetime.now()

            mac = ':'.join(f'{byte:02x}' for byte in mac_bytes)
            return {
                'meta': {
                    'len': csi_len,
                    'mac': mac,
                    'rssi': rssi,
                    'agc': agc,
                    'fft': fft,
                    'comp': comp,
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
        if self.reference_rssi is None:
            self.reference_rssi = csi_data['rssi']
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

        # Prefer firmware-provided gain compensation. ESP-IDF builds without
        # esp_csi_gain_ctrl_* fall back to RSSI-relative normalization.
        comp = self.compensation_gains[-1] if self.compensation_gains else 0
        if comp == 0 and self.last_meta and self.reference_rssi is not None:
            comp = self.last_meta['rssi'] - self.reference_rssi
            if self.compensation_gains:
                self.compensation_gains[-1] = comp
        self.last_applied_comp = comp
        corrected_db = raw_db - comp
        corrected_subcarriers = 10 ** (corrected_db / 20.0)

        self.csi_raw.append(raw_subcarriers)
        self.csi_corrected.append(corrected_subcarriers)
        self.csi_raw_db.append(raw_db)
        self.csi_corrected_db.append(corrected_db)
        self.csi_raw_db_history.append(raw_db)
        self.csi_time_history.append(self.timestamps[-1] if self.timestamps else 0.0)
        self.last_csi_len = len(csi_bytes)

        if self.frame_count <= 3 or self.frame_count % 25 == 0:
            print(
                f"     CSI bytes:{len(csi_bytes):4d} subcarriers:{raw_subcarriers.size:3d} "
                f"raw:{raw_db.min():6.1f}..{raw_db.max():6.1f} dB "
                f"corr:{corrected_db.min():6.1f}..{corrected_db.max():6.1f} dB "
                f"applied_comp:{comp:5.1f} dB"
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


def recent_raw_history(monitor):
    times = list(monitor.csi_time_history)
    raw_frames = list(monitor.csi_raw_db_history)
    if not times or not raw_frames:
        return []

    start_time = times[-1] - monitor.plot_window
    selected = [
        frame
        for time_value, frame in zip(times, raw_frames)
        if time_value >= start_time
    ]
    return selected or raw_frames[-1:]


def pca_motion_signal(matrix):
    clean = np.array(matrix, dtype=np.float32)
    row_means = np.nanmean(clean, axis=1, keepdims=True)
    row_means = np.where(np.isfinite(row_means), row_means, 0.0)
    clean = np.where(np.isfinite(clean), clean, row_means)
    centered = clean - np.mean(clean, axis=1, keepdims=True)
    if centered.shape[1] < 2:
        return np.zeros(centered.shape[1], dtype=np.float32)

    try:
        _u, _s, vt = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return np.nanmean(centered, axis=0)

    signal = vt[0] * _s[0]
    if np.abs(np.nanmin(signal)) > np.abs(np.nanmax(signal)):
        signal = -signal
    return signal.astype(np.float32)


def lowpass(signal, window=11):
    if signal.size == 0:
        return signal
    window = min(window, max(1, signal.size))
    if window % 2 == 0:
        window -= 1
    if window <= 1:
        return signal
    kernel = np.ones(window, dtype=np.float32) / window
    padded = np.pad(signal, (window // 2, window // 2), mode='edge')
    return np.convolve(padded, kernel, mode='valid')


def create_visualization(monitor):
    fig, (ax_raw, ax_pca, ax_lpf) = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.88, bottom=0.13, hspace=0.34)

    n_display = 96
    raw_lines = []
    cmap_raw = plt.cm.viridis
    for i in range(n_display):
        color_r = cmap_raw(i / max(1, n_display - 1))
        (lr,) = ax_raw.plot([], [], color=color_r, linewidth=0.9, alpha=0.9)
        raw_lines.append(lr)
    (pca_line,) = ax_pca.plot([], [], color='#1f77b4', linewidth=1.4)
    (lpf_line,) = ax_lpf.plot([], [], color='#1f77b4', linewidth=1.4)

    ax_raw.set_title('Original Signal', fontsize=12)
    ax_raw.set_xlabel('')
    ax_raw.set_ylabel('CSI amplitude (dB rel.)')
    ax_pca.set_title('PCA Filtered Signal', fontsize=12)
    ax_pca.set_ylabel('Amplitude')
    ax_lpf.set_title('LPF Filtered Signal', fontsize=12)
    ax_lpf.set_xlabel('Number of Packets')
    ax_lpf.set_ylabel('Amplitude')
    for ax in (ax_raw, ax_pca, ax_lpf):
        ax.grid(True, alpha=0.25)

    status_text = fig.text(0.08, 0.92, 'Waiting for CSI...', fontsize=11)
    subcarrier_text = fig.text(0.08, 0.885, 'Subcarriers affiches: attente CSI...', fontsize=10)
    warning_text = fig.text(0.08, 0.055, '', fontsize=10, color='#9a3412')
    fig.suptitle('ESP32 CSI', fontsize=17, fontweight='bold')

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
        for _ in range(250):
            had_data = monitor.ser.in_waiting > 0 or bool(monitor.serial_buffer)
            monitor.read_serial()
            if not had_data:
                break
        raw_history = recent_raw_history(monitor)
        if len(raw_history) > 1:
            raw = history_to_matrix(raw_history)
            frames = raw.shape[1]
            subcarriers = raw.shape[0]

            n_plot = min(n_display, subcarriers)
            if subcarriers > n_plot:
                start = max(0, subcarriers // 2 - n_plot // 2)
            else:
                start = 0
            indices = list(range(start, start + n_plot))

            x = np.arange(frames, dtype=np.float32)
            for line_idx, sc_idx in enumerate(indices):
                y_raw = raw[sc_idx, :]
                raw_lines[line_idx].set_data(x, y_raw)

            for line in raw_lines[n_plot:]:
                line.set_data([], [])

            pca = pca_motion_signal(raw)
            lpf = lowpass(pca)
            pca_line.set_data(x, pca)
            lpf_line.set_data(x, lpf)

            ax_raw.set_xlim(0, max(1, frames - 1))
            ax_raw.set_ylim(*monitor.robust_limits(raw))
            ax_pca.set_xlim(0, max(1, frames - 1))
            ax_lpf.set_xlim(0, max(1, frames - 1))
            ax_pca.set_ylim(*monitor.robust_limits(pca))
            ax_lpf.set_ylim(*monitor.robust_limits(lpf))

            if monitor.last_meta:
                status_text.set_text(
                    f"Frames: {monitor.frame_count} | CSI bytes: {monitor.last_csi_len} | "
                    f"Affichage: {frames} paquets | Subcarriers: {subcarriers} | RSSI: {monitor.last_meta['rssi']} dBm | "
                    f"MAC: {monitor.last_meta['mac']} | filtres: {monitor.filtered_count}"
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
        return tuple(raw_lines) + (pca_line, lpf_line, status_text, subcarrier_text, warning_text)

    ani = FuncAnimation(fig, animate, interval=100, blit=False, cache_frame_data=False)
    fig._csi_animation = ani
    fig._csi_buttons = (button_details, button_frames)

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='CSI Monitor - real-time visualization from ESP32')
    parser.add_argument('-p', '--port', default='COM23', help='Serial port (default: COM23)')
    parser.add_argument('-b', '--baud', type=int, default=2000000, help='Baud rate (default: 2000000)')
    parser.add_argument('-s', '--size', type=int, default=100, help='Metric buffer size (default: 100)')
    parser.add_argument('--history', type=int, default=5000, help='CSI packets kept in memory (default: 5000)')
    parser.add_argument('--window', type=float, default=5.0, help='CSI plot time window in seconds (default: 5.0)')
    parser.add_argument('--mac', action='append', default=[], help='Only plot/count CSI frames from this MAC. Can be repeated.')
    args = parser.parse_args()
    history_size = max(args.history, int(args.window * 1000))

    monitor = CSIMonitor(
        port=args.port,
        baudrate=args.baud,
        buffer_size=args.size,
        csi_history_size=history_size,
        plot_window=args.window,
        mac_filter=args.mac,
    )

    print("\n" + "=" * 60)
    print("ESP32 CSI Real-Time Monitor")
    print("=" * 60)
    print(f"Port: {args.port}, Baudrate: {args.baud}")
    print(f"CSI display window: last {args.window:.1f} seconds")
    print(f"CSI history buffer: {history_size} packets")
    if args.mac:
        print(f"MAC filter: {', '.join(args.mac)}")
    print("Main window shows original CSI, PCA signal, and LPF signal.")
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
