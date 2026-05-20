#!/usr/bin/env python3
"""
Clean CSI monitor for the ZoeFall RX AMP stream.

Expected RX line format:
    AMP:<seq>:<agc_gain>:<fft_gain>:<comp_gain>:a0,a1,...,a63

Legacy RX line format is also accepted:
    AMP:<seq>:a0,a1,...,a63

Windows shown:
    1. Raw CSI and AGC-corrected CSI.
    2. PCA raw and PCA AGC-corrected.
"""

import argparse
import sys
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, CheckButtons


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
        self.comp_gains = deque(maxlen=self.max_frames)
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
        self.comp_gains.append(comp)
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
            comp = self.comp_gains[-1] if self.comp_gains else 1.0
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

    def comp_gain_vector(self):
        if not self.comp_gains:
            return np.empty((0,), dtype=np.float32)
        return np.asarray(list(self.comp_gains), dtype=np.float32)

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


def comp_gain_delta_db(comp_gain):
    comp = np.asarray(comp_gain, dtype=np.float32)
    comp = np.where(np.isfinite(comp) & (comp > 0.0), comp, 1.0)
    return 20.0 * np.log10(comp)


def apply_comp_gain(raw_amp, comp_gain):
    if raw_amp.size == 0:
        return raw_amp

    comp = np.asarray(comp_gain, dtype=np.float32)
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
    fig_csi.subplots_adjust(left=0.07, right=0.98, top=0.91, bottom=0.20, hspace=0.28)
    fig_csi.suptitle("CSI raw / CSI corrigee", fontsize=15, fontweight="bold")

    fig_pca, (ax_pca_raw, ax_pca_corr) = plt.subplots(2, 1, figsize=(15, 7), sharex=True)
    fig_pca.subplots_adjust(left=0.07, right=0.98, top=0.86, bottom=0.12, hspace=0.32)
    fig_pca.suptitle("PCA raw / PCA corrigee", fontsize=15, fontweight="bold")

    raw_lines = []
    corr_lines = []

    (pca_raw_line,) = ax_pca_raw.plot([], [], color="#1f77b4", lw=1.8)
    (pca_corr_line,) = ax_pca_corr.plot([], [], color="#d62728", lw=1.8)

    status = fig_csi.text(0.07, 0.92, "Waiting for AMP frames...", fontsize=10)
    subcarrier_text = fig_csi.text(0.07, 0.895, "", fontsize=9)
    paused_text = fig_csi.text(0.80, 0.92, "", fontsize=10, color="#b91c1c", fontweight="bold")

    controls = {
        "paused": False,
        "selected": None,
        "selector_fig": None,
        "check_buttons": [],
        "updating_checks": False,
        "last_subcarrier_count": 0,
    }

    pause_ax = fig_csi.add_axes([0.07, 0.055, 0.09, 0.045])
    select_ax = fig_csi.add_axes([0.18, 0.055, 0.16, 0.045])
    all_ax = fig_csi.add_axes([0.36, 0.055, 0.13, 0.045])
    pause_button = Button(pause_ax, "Pause")
    select_button = Button(select_ax, "Sous-carrieres")
    all_button = Button(all_ax, "Tout afficher")

    ax_raw.set_title("CSI raw")
    ax_raw.set_ylabel("Amplitude (dB)")
    ax_corr.set_title("CSI corrigee AGC")
    ax_corr.set_ylabel("Amplitude (dB)")
    ax_corr.set_xlabel("Number of packets")

    ax_pca_raw.set_title("PCA raw")
    ax_pca_raw.set_ylabel("PC1")
    ax_pca_corr.set_title("PCA CSI corrigee AGC")
    ax_pca_corr.set_ylabel("PC1")
    ax_pca_corr.set_xlabel("Number of packets")

    for ax in (ax_raw, ax_corr, ax_pca_raw, ax_pca_corr):
        ax.grid(True, alpha=0.25)

    def set_pause(_event=None):
        controls["paused"] = not controls["paused"]
        pause_button.label.set_text("Reprendre" if controls["paused"] else "Pause")
        paused_text.set_text("PAUSE" if controls["paused"] else "")
        fig_csi.canvas.draw_idle()

    def ensure_lines(n_lines):
        if len(raw_lines) >= n_lines:
            return
        colors = plt.cm.tab20(np.linspace(0, 1, max(1, n_lines)))
        for i in range(len(raw_lines), n_lines):
            color = colors[i % len(colors)]
            (raw_line,) = ax_raw.plot([], [], lw=0.75, alpha=0.85, color=color)
            (corr_line,) = ax_corr.plot([], [], lw=0.75, alpha=0.85, color=color)
            raw_lines.append(raw_line)
            corr_lines.append(corr_line)

    def active_subcarriers(n_subcarriers):
        selected = controls["selected"]
        if selected is None:
            selected = set(choose_subcarriers(n_subcarriers, monitor.max_subcarriers))
            controls["selected"] = selected
        return sorted(sc for sc in selected if 0 <= sc < n_subcarriers)

    def refresh_check_buttons():
        if not controls["check_buttons"] or controls["selected"] is None:
            return
        controls["updating_checks"] = True
        for checks in controls["check_buttons"]:
            for i, label in enumerate(checks.labels):
                sc = int(label.get_text())
                wanted = sc in controls["selected"]
                if checks.get_status()[i] != wanted:
                    checks.set_active(i)
        controls["updating_checks"] = False

    def set_all_subcarriers(_event=None):
        n_subcarriers = controls["last_subcarrier_count"]
        if n_subcarriers <= 0:
            return
        controls["selected"] = set(range(n_subcarriers))
        refresh_check_buttons()
        fig_csi.canvas.draw_idle()

    def open_selector(_event=None):
        n_subcarriers = controls["last_subcarrier_count"]
        if n_subcarriers <= 0:
            return
        if controls["selected"] is None:
            controls["selected"] = set(choose_subcarriers(n_subcarriers, monitor.max_subcarriers))
        if controls["selector_fig"] is not None and plt.fignum_exists(controls["selector_fig"].number):
            controls["selector_fig"].canvas.manager.show()
            controls["selector_fig"].canvas.draw_idle()
            return

        selector_fig = plt.figure(figsize=(8, 8))
        selector_fig.suptitle("Sous-carrieres affichees", fontsize=13, fontweight="bold")
        controls["selector_fig"] = selector_fig
        controls["check_buttons"] = []

        cols = 4 if n_subcarriers > 32 else 2
        rows_per_col = int(np.ceil(n_subcarriers / cols))
        for col in range(cols):
            start = col * rows_per_col
            stop = min(start + rows_per_col, n_subcarriers)
            if start >= stop:
                continue
            labels = [str(i) for i in range(start, stop)]
            actives = [i in controls["selected"] for i in range(start, stop)]
            ax_checks = selector_fig.add_axes([
                0.06 + col * (0.88 / cols),
                0.16,
                0.82 / cols,
                0.74,
            ])
            checks = CheckButtons(ax_checks, labels, actives)

            def on_check(label):
                if controls["updating_checks"]:
                    return
                sc = int(label)
                if sc in controls["selected"]:
                    controls["selected"].remove(sc)
                else:
                    controls["selected"].add(sc)
                fig_csi.canvas.draw_idle()

            checks.on_clicked(on_check)
            controls["check_buttons"].append(checks)

        selector_all_ax = selector_fig.add_axes([0.18, 0.045, 0.18, 0.055])
        selector_none_ax = selector_fig.add_axes([0.42, 0.045, 0.18, 0.055])
        selector_default_ax = selector_fig.add_axes([0.66, 0.045, 0.18, 0.055])
        selector_all = Button(selector_all_ax, "Tout")
        selector_none = Button(selector_none_ax, "Aucun")
        selector_default = Button(selector_default_ax, "Defaut")

        def select_none(_button_event=None):
            controls["selected"] = set()
            refresh_check_buttons()
            fig_csi.canvas.draw_idle()

        def select_default(_button_event=None):
            controls["selected"] = set(choose_subcarriers(n_subcarriers, monitor.max_subcarriers))
            refresh_check_buttons()
            fig_csi.canvas.draw_idle()

        selector_all.on_clicked(set_all_subcarriers)
        selector_none.on_clicked(select_none)
        selector_default.on_clicked(select_default)
        selector_fig._csi_selector_widgets = (
            controls["check_buttons"],
            selector_all,
            selector_none,
            selector_default,
        )
        selector_fig.show()

    pause_button.on_clicked(set_pause)
    select_button.on_clicked(open_selector)
    all_button.on_clicked(set_all_subcarriers)

    def animate(_):
        if controls["paused"]:
            return tuple(raw_lines + corr_lines + [pca_raw_line, pca_corr_line, status, subcarrier_text, paused_text])

        for _i in range(20):
            if not monitor.read_available():
                break

        raw_amp = monitor.raw_matrix()
        if raw_amp.shape[0] < 2:
            return tuple(raw_lines + corr_lines + [pca_raw_line, pca_corr_line, status, subcarrier_text, paused_text])

        comp_gain = monitor.comp_gain_vector()
        agc_values = monitor.agc_vector()
        fft_values = monitor.fft_vector()
        corrected_amp = apply_comp_gain(raw_amp, comp_gain)
        raw = monitor.amp_to_db(raw_amp)
        corr = monitor.amp_to_db(corrected_amp)
        frames, subcarriers = raw_amp.shape
        if frames > max_plot_points:
            plot_raw = raw[-max_plot_points:]
            plot_corr = corr[-max_plot_points:]
            plot_comp_gain = comp_gain[-max_plot_points:]
            plot_agc = agc_values[-max_plot_points:]
            plot_fft = fft_values[-max_plot_points:]
        else:
            plot_raw = raw
            plot_corr = corr
            plot_comp_gain = comp_gain
            plot_agc = agc_values
            plot_fft = fft_values
        plot_frames = plot_raw.shape[0]
        controls["last_subcarrier_count"] = subcarriers
        indices = active_subcarriers(subcarriers)
        ensure_lines(len(indices))
        x = np.arange(plot_frames, dtype=np.float32)

        for line_i, sc_i in enumerate(indices):
            raw_lines[line_i].set_data(x, plot_raw[:, sc_i])
            corr_lines[line_i].set_data(x, plot_corr[:, sc_i])
        for line in raw_lines[len(indices):]:
            line.set_data([], [])
        for line in corr_lines[len(indices):]:
            line.set_data([], [])

        if indices:
            pca_raw = pca_first_component(plot_raw[:, indices])
            pca_corr = pca_first_component(plot_corr[:, indices])
        else:
            pca_raw = np.zeros(plot_frames, dtype=np.float32)
            pca_corr = np.zeros(plot_frames, dtype=np.float32)
        pca_raw_line.set_data(x, pca_raw)
        pca_corr_line.set_data(x, pca_corr)

        for ax in (ax_raw, ax_corr, ax_pca_raw, ax_pca_corr):
            ax.set_xlim(0, max(1, plot_frames - 1))
        if indices:
            csi_ylim = robust_limits(np.concatenate((plot_raw[:, indices], plot_corr[:, indices]), axis=0),
                                     padding=db_padding)
        else:
            csi_ylim = (0.0, 1.0)
        ax_raw.set_ylim(*csi_ylim)
        ax_corr.set_ylim(*csi_ylim)
        ax_pca_raw.set_ylim(*robust_limits(pca_raw, padding=0.25))
        ax_pca_corr.set_ylim(*robust_limits(pca_corr, padding=0.25))

        py_rate = monitor.rate_hz()
        seq_rate = monitor.seq_rate_hz()
        py_text = f"{py_rate:.1f}" if py_rate is not None else "n/a"
        seq_text = f"{seq_rate:.1f}" if seq_rate is not None else "n/a"
        latest_comp = plot_comp_gain[-1] if np.isfinite(plot_comp_gain[-1]) and plot_comp_gain[-1] > 0.0 else 1.0
        latest_agc = int(plot_agc[-1]) if plot_agc.size else 0
        latest_fft = int(plot_fft[-1]) if plot_fft.size else 0
        delta_db = comp_gain_delta_db(plot_comp_gain)
        latest_delta = delta_db[-1] if delta_db.size else 0.0
        status.set_text(
            f"Frames: {monitor.total} | affichage: {frames} paquets | "
            f"py: {py_text} Hz | seq: {seq_text} Hz | "
            f"drop: {monitor.dropped_seq} | reset: {monitor.seq_resets} | bad: {monitor.bad_lines} | "
            f"agc: {latest_agc} | fft: {latest_fft} | "
            f"comp: {latest_comp:.4f} | delta: {latest_delta:+.2f} dB"
        )
        if seq_rate is not None and abs(seq_rate - monitor.target_hz) > monitor.target_hz * 0.15:
            status.set_color("#b91c1c")
        else:
            status.set_color("#111827")
        if indices:
            if len(indices) == subcarriers:
                selected_text = f"0..{subcarriers - 1}"
            else:
                selected_text = ", ".join(str(i) for i in indices[:18])
                if len(indices) > 18:
                    selected_text += ", ..."
            subcarrier_text.set_text(
                f"Subcarriers affichees: {selected_text} ({len(indices)}/{subcarriers})"
            )
        else:
            subcarrier_text.set_text(f"Subcarriers affichees: aucune (0/{subcarriers})")

        fig_pca.canvas.draw_idle()
        return tuple(raw_lines + corr_lines + [pca_raw_line, pca_corr_line, status, subcarrier_text, paused_text])

    ani = FuncAnimation(fig_csi, animate, interval=50, blit=False, cache_frame_data=False)
    fig_csi._csi_animation = ani
    fig_csi._csi_widgets = (pause_button, select_button, all_button)
    fig_pca._csi_animation = ani
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Clean CSI monitor for AMP:<seq>:... streams")
    parser.add_argument("-p", "--port", default="COM16", help="Serial port")
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
