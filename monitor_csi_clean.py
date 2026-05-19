#!/usr/bin/env python3
"""
Clean CSI monitor for the ZoeFall RX AMP stream.

Expected RX line format:
    AMP:<seq>:<agc_gain>:<fft_gain>:<comp_gain>:a0,a1,...,a63

Legacy RX line format is also accepted:
    AMP:<seq>:a0,a1,...,a63

Windows shown:
    1. Raw CSI and gain-corrected CSI.
    2. PCA raw and PCA corrected.
"""

import argparse
import sys
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation


AMP_PREFIX = b"AMP:"


class CleanCSIMonitor:
    def __init__(self, port, baud, window_s, target_hz, max_subcarriers, print_every):
        self.port = port
        self.baud = baud
        self.window_s = window_s
        self.target_hz = target_hz
        self.max_frames = max(32, int(window_s * target_hz * 1.5))
        self.max_subcarriers = max_subcarriers
        self.print_every = print_every

        self.raw_amp = deque(maxlen=self.max_frames)
        self.raw_db = deque(maxlen=self.max_frames)
        self.agc_gains = deque(maxlen=self.max_frames)
        self.fft_gains = deque(maxlen=self.max_frames)
        self.agc_correction = deque(maxlen=self.max_frames)
        self.seq = deque(maxlen=self.max_frames)
        self.host_t = deque(maxlen=self.max_frames)
        self.rate_events = deque(maxlen=max(16, int(target_hz * 3)))

        self.buffer = bytearray()
        self.total = 0
        self.dropped_seq = 0
        self.bad_lines = 0
        self.last_seq = None
        self.seq_resets = 0

        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
        except Exception as exc:
            print(f"Failed to open {port}: {exc}")
            sys.exit(1)

        print(f"Connected to {port} at {baud} baud")
        print(f"Display window: {window_s:.1f}s, target: {target_hz:.1f} Hz")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")

    @staticmethod
    def amp_to_db(amps):
        values = np.asarray(amps, dtype=np.float32)
        db = np.full(values.shape, np.nan, dtype=np.float32)
        valid = values > 0
        db[valid] = 20.0 * np.log10(values[valid])
        return db

    @staticmethod
    def parse_amp_line(line):
        start = line.find(AMP_PREFIX)
        if start < 0:
            return None
        parts = line[start + len(AMP_PREFIX):].split(b":", 4)

        try:
            seq = int(parts[0])
            if len(parts) == 5:
                agc = int(parts[1])
                fft = int(parts[2])
                comp = float(parts[3])
                amp_field = parts[4]
            elif len(parts) == 3:
                agc = 0
                fft = 0
                comp = float(parts[1])
                amp_field = parts[2]
            elif len(parts) == 2:
                agc = 0
                fft = 0
                comp = 1.0
                amp_field = parts[1]
            else:
                return None
            amps = [int(part) for part in amp_field.split(b",") if part.strip()]
        except ValueError:
            return None

        if len(amps) < 8:
            return None
        return seq, agc, fft, comp, amps

    def read_available(self):
        if self.ser.in_waiting:
            self.buffer.extend(self.ser.read(self.ser.in_waiting))

        parsed = 0
        while True:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                if len(self.buffer) > 8192:
                    del self.buffer[:-256]
                break

            line = bytes(self.buffer[:newline]).strip()
            del self.buffer[:newline + 1]
            parsed += self.process_line(line)

        return parsed

    def process_line(self, line):
        parsed = self.parse_amp_line(line)
        if parsed is None:
            if b"AMP:" in line:
                self.bad_lines += 1
            return 0

        seq, agc, fft, comp, amps = parsed
        amp = np.asarray(amps, dtype=np.float32)
        db = self.amp_to_db(amps)

        if self.last_seq is not None:
            gap = (seq - self.last_seq) & 0xffff
            if gap > 30000:
                self.seq_resets += 1
                self.rate_events.clear()
            elif gap > 1:
                self.dropped_seq += gap - 1
        self.last_seq = seq

        now = time.monotonic()
        self.raw_amp.append(amp)
        self.raw_db.append(db)
        self.agc_gains.append(agc)
        self.fft_gains.append(fft)
        self.agc_correction.append(comp)
        self.seq.append(seq)
        self.host_t.append(now)
        self.rate_events.append((now, seq))
        self.total += 1

        if self.print_every > 0 and (self.total <= 3 or self.total % self.print_every == 0):
            self.print_status()
        return 1

    def rate_hz(self):
        if len(self.rate_events) < 2:
            return None
        span = self.rate_events[-1][0] - self.rate_events[0][0]
        if span <= 0:
            return None
        return (len(self.rate_events) - 1) / span

    def seq_rate_hz(self):
        if len(self.rate_events) < 2:
            return None
        span = self.rate_events[-1][0] - self.rate_events[0][0]
        if span <= 0:
            return None
        seq_delta = 0
        prev_seq = self.rate_events[0][1]
        for _t, cur_seq in list(self.rate_events)[1:]:
            gap = (cur_seq - prev_seq) & 0xffff
            if gap > 30000:
                gap = 1
            seq_delta += gap
            prev_seq = cur_seq
        return seq_delta / span

    def print_status(self):
        py_rate = self.rate_hz()
        seq_rate = self.seq_rate_hz()
        py_text = f"{py_rate:6.1f}" if py_rate is not None else "   n/a"
        seq_text = f"{seq_rate:6.1f}" if seq_rate is not None else "   n/a"
        latest = self.raw_db[-1] if self.raw_db else np.array([])
        finite = latest[np.isfinite(latest)]
        if finite.size:
            agc = self.agc_gains[-1] if self.agc_gains else 0
            fft = self.fft_gains[-1] if self.fft_gains else 0
            comp = self.agc_correction[-1] if self.agc_correction else 1.0
            comp_db = 20.0 * np.log10(comp) if comp > 0.0 else 0.0
            limits = (
                f"{finite.min():5.1f}..{finite.max():5.1f} dB "
                f"agc:{agc} fft:{fft} corr:{comp:.4f}/{comp_db:+.2f}dB"
            )
        else:
            limits = "no CSI"
        print(
            f"[{self.total:6d}] py:{py_text} Hz seq:{seq_text} Hz "
            f"drop:{self.dropped_seq} reset:{self.seq_resets} bad:{self.bad_lines} CSI:{len(latest):3d} {limits}"
        )
        if seq_rate is not None and abs(seq_rate - self.target_hz) > self.target_hz * 0.15:
            print(f"     WARNING: seq rate is not close to {self.target_hz:.1f} Hz")

    def raw_matrix(self):
        if not self.raw_amp:
            return np.empty((0, 0), dtype=np.float32)

        rows = list(self.raw_amp)
        max_len = max(len(row) for row in rows)
        mat = np.full((len(rows), max_len), np.nan, dtype=np.float32)
        for i, row in enumerate(rows):
            mat[i, :len(row)] = row
        return mat

    def agc_correction_vector(self):
        if not self.agc_correction:
            return np.empty((0,), dtype=np.float32)
        return np.asarray(list(self.agc_correction), dtype=np.float32)

    def agc_vector(self):
        if not self.agc_gains:
            return np.empty((0,), dtype=np.int16)
        return np.asarray(list(self.agc_gains), dtype=np.int16)

    def fft_vector(self):
        if not self.fft_gains:
            return np.empty((0,), dtype=np.int16)
        return np.asarray(list(self.fft_gains), dtype=np.int16)


def robust_limits(data, padding=1.5):
    finite = data[np.isfinite(data)]
    if finite.size == 0:
        return 0.0, 1.0
    lo, hi = np.percentile(finite, [2, 98])
    if lo == hi:
        hi = lo + 1.0
    return float(lo - padding), float(hi + padding)


def apply_agc_correction(raw_amp, agc_correction):
    if raw_amp.size == 0:
        return raw_amp

    comp = np.asarray(agc_correction, dtype=np.float32)
    if comp.size < raw_amp.shape[0]:
        comp = np.pad(comp, (raw_amp.shape[0] - comp.size, 0), constant_values=1.0)
    comp = comp[-raw_amp.shape[0]:]
    comp = np.where(np.isfinite(comp) & (comp > 0.0), comp, 1.0)
    return raw_amp * comp.reshape(-1, 1)


def pca_first_component(frames_by_subcarrier):
    if frames_by_subcarrier.shape[0] < 2 or frames_by_subcarrier.shape[1] < 2:
        return np.zeros(frames_by_subcarrier.shape[0], dtype=np.float32)

    x = np.asarray(frames_by_subcarrier, dtype=np.float32)
    col_mean = np.nanmean(x, axis=0, keepdims=True)
    col_mean = np.where(np.isfinite(col_mean), col_mean, 0.0)
    x = np.where(np.isfinite(x), x, col_mean)

    x -= np.mean(x, axis=0, keepdims=True)
    col_std = np.std(x, axis=0, keepdims=True)
    x = x / np.where(col_std > 1e-6, col_std, 1.0)

    try:
        _u, _s, vt = np.linalg.svd(x, full_matrices=False)
        signal = x @ vt[0]
    except np.linalg.LinAlgError:
        signal = np.nanmean(x, axis=1)

    signal -= np.nanmean(signal)
    if abs(np.nanmin(signal)) > abs(np.nanmax(signal)):
        signal = -signal
    return signal.astype(np.float32)


def moving_average(signal, window=9):
    if signal.size == 0:
        return signal
    w = min(window, signal.size)
    if w % 2 == 0:
        w -= 1
    if w <= 1:
        return signal
    kernel = np.ones(w, dtype=np.float32) / w
    padded = np.pad(signal, (w // 2, w // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def choose_subcarriers(n_subcarriers, max_display):
    n = min(n_subcarriers, max_display)
    if n_subcarriers <= n:
        return list(range(n_subcarriers))
    start = n_subcarriers // 2 - n // 2
    return list(range(start, start + n))


def create_plots(monitor, max_plot_points, db_padding=15):
    fig_csi, (ax_raw, ax_corr) = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
    fig_csi.subplots_adjust(left=0.07, right=0.98, top=0.95, bottom=0.20, hspace=0.28)
    fig_csi.suptitle("CSI raw / CSI corrigee", fontsize=15, fontweight="bold")

    fig_pca, (ax_pca_raw, ax_pca_corr) = plt.subplots(2, 1, figsize=(15, 7), sharex=True)
    fig_pca.subplots_adjust(left=0.07, right=0.98, top=0.86, bottom=0.12, hspace=0.32)
    fig_pca.suptitle("PCA raw / PCA corrigee", fontsize=15, fontweight="bold")

    raw_lines = []
    corr_lines = []
    colors = plt.cm.tab20(np.linspace(0, 1, monitor.max_subcarriers))
    for i in range(monitor.max_subcarriers):
        (raw_line,) = ax_raw.plot([], [], lw=0.75, alpha=0.85, color=colors[i])
        (corr_line,) = ax_corr.plot([], [], lw=0.75, alpha=0.85, color=colors[i])
        raw_lines.append(raw_line)
        corr_lines.append(corr_line)

    (pca_raw_line,) = ax_pca_raw.plot([], [], color="#1f77b4", lw=1.8)
    (pca_corr_line,) = ax_pca_corr.plot([], [], color="#d62728", lw=1.8)

    status = fig_csi.text(0.07, 0.92, "Waiting for AMP frames...", fontsize=10)
    subcarrier_text = fig_csi.text(0.07, 0.895, "", fontsize=9)

    ax_raw.set_title("CSI raw")
    ax_raw.set_ylabel("Amplitude (dB)")
    ax_corr.set_title("CSI brute * correction AGC")
    ax_corr.set_ylabel("Amplitude (dB)")
    ax_corr.set_xlabel("Number of packets")

    ax_pca_raw.set_title("PCA raw")
    ax_pca_raw.set_ylabel("PC1")
    ax_pca_corr.set_title("PCA CSI * correction AGC")
    ax_pca_corr.set_ylabel("PC1")
    ax_pca_corr.set_xlabel("Number of packets")

    for ax in (ax_raw, ax_corr, ax_pca_raw, ax_pca_corr):
        ax.grid(True, alpha=0.25)

    def animate(_):
        for _i in range(20):
            if not monitor.read_available():
                break

        raw_amp = monitor.raw_matrix()
        if raw_amp.shape[0] < 2:
            return tuple(raw_lines + corr_lines + [pca_raw_line, pca_corr_line, status, subcarrier_text])

        agc_corr = monitor.agc_correction_vector()
        agc_values = monitor.agc_vector()
        fft_values = monitor.fft_vector()
        corrected_amp = apply_agc_correction(raw_amp, agc_corr)
        raw = monitor.amp_to_db(raw_amp)
        corr = monitor.amp_to_db(corrected_amp)
        frames, subcarriers = raw_amp.shape
        if frames > max_plot_points:
            plot_raw = raw[-max_plot_points:]
            plot_corr = corr[-max_plot_points:]
            plot_agc_corr = agc_corr[-max_plot_points:]
            plot_agc = agc_values[-max_plot_points:]
            plot_fft = fft_values[-max_plot_points:]
        else:
            plot_raw = raw
            plot_corr = corr
            plot_agc_corr = agc_corr
            plot_agc = agc_values
            plot_fft = fft_values
        plot_frames = plot_raw.shape[0]
        indices = choose_subcarriers(subcarriers, monitor.max_subcarriers)
        x = np.arange(plot_frames, dtype=np.float32)

        for line_i, sc_i in enumerate(indices):
            raw_lines[line_i].set_data(x, plot_raw[:, sc_i])
            corr_lines[line_i].set_data(x, plot_corr[:, sc_i])
        for line in raw_lines[len(indices):]:
            line.set_data([], [])
        for line in corr_lines[len(indices):]:
            line.set_data([], [])

        pca_raw = pca_first_component(plot_raw[:, indices])
        pca_corr = pca_first_component(plot_corr[:, indices])
        pca_raw_line.set_data(x, pca_raw)
        pca_corr_line.set_data(x, pca_corr)

        for ax in (ax_raw, ax_corr, ax_pca_raw, ax_pca_corr):
            ax.set_xlim(0, max(1, plot_frames - 1))
        ax_raw.set_ylim(*robust_limits(plot_raw[:, indices], padding=db_padding))
        ax_corr.set_ylim(*robust_limits(plot_corr[:, indices], padding=db_padding))
        ax_pca_raw.set_ylim(*robust_limits(pca_raw, padding=0.25))
        ax_pca_corr.set_ylim(*robust_limits(pca_corr, padding=0.25))

        py_rate = monitor.rate_hz()
        seq_rate = monitor.seq_rate_hz()
        py_text = f"{py_rate:.1f}" if py_rate is not None else "n/a"
        seq_text = f"{seq_rate:.1f}" if seq_rate is not None else "n/a"
        latest_corr = plot_agc_corr[-1] if np.isfinite(plot_agc_corr[-1]) and plot_agc_corr[-1] > 0.0 else 1.0
        latest_agc = int(plot_agc[-1]) if plot_agc.size else 0
        latest_fft = int(plot_fft[-1]) if plot_fft.size else 0
        status.set_text(
            f"Frames: {monitor.total} | affichage: {frames} paquets | "
            f"py: {py_text} Hz | seq: {seq_text} Hz | "
            f"drop: {monitor.dropped_seq} | reset: {monitor.seq_resets} | bad: {monitor.bad_lines} | "
            f"agc: {latest_agc} | fft: {latest_fft} | "
            f"corr: {latest_corr:.4f} ({20.0 * np.log10(latest_corr):+.2f} dB)"
        )
        if seq_rate is not None and abs(seq_rate - monitor.target_hz) > monitor.target_hz * 0.15:
            status.set_color("#b91c1c")
        else:
            status.set_color("#111827")
        subcarrier_text.set_text(
            f"Subcarriers affichees: {indices[0]}..{indices[-1]} ({len(indices)}/{subcarriers})"
        )

        fig_pca.canvas.draw_idle()
        return tuple(raw_lines + corr_lines + [pca_raw_line, pca_corr_line, status, subcarrier_text])

    ani = FuncAnimation(fig_csi, animate, interval=50, blit=False, cache_frame_data=False)
    fig_csi._csi_animation = ani
    fig_pca._csi_animation = ani
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Clean CSI monitor for AMP:<seq>:... streams")
    parser.add_argument("-p", "--port", default="COM25", help="Serial port")
    parser.add_argument("-b", "--baud", type=int, default=2000000, help="Serial baudrate")
    parser.add_argument("--window", type=float, default=5.0, help="Display window in seconds")
    parser.add_argument("--target-hz", type=float, default=100.0, help="Expected CSI rate")
    parser.add_argument("--subcarriers", type=int, default=64, help="Max subcarriers to plot")
    parser.add_argument("--max-plot-points", type=int, default=600, help="Max points drawn per line")
    parser.add_argument("--print-every", type=int, default=100, help="Print status every N frames")
    parser.add_argument("--db-padding", type=float, default=15, help="Padding (dB) added to computed y-limits for CSI plots")
    args = parser.parse_args()

    monitor = CleanCSIMonitor(
        port=args.port,
        baud=args.baud,
        window_s=args.window,
        target_hz=args.target_hz,
        max_subcarriers=args.subcarriers,
        print_every=args.print_every,
    )

    try:
        create_plots(monitor, args.max_plot_points, db_padding=args.db_padding)
    except KeyboardInterrupt:
        print("\nMonitoring stopped")
    finally:
        monitor.close()


if __name__ == "__main__":
    main()
