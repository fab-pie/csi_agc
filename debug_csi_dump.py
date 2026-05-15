#!/usr/bin/env python3
"""Debug helper: dump raw CSI lines and try multiple signed-width interpretations.
Usage: python debug_csi_dump.py COM16 115200
"""
import sys
import serial
import math
from collections import Counter


def to_signed(v, bits):
    mask = 2 ** bits
    if v >= mask // 2:
        v = v - mask
    return v


def amps_from_tokens(tokens, width_bits):
    vals = []
    for t in tokens:
        try:
            v = int(t)
        except Exception:
            try:
                v = int(t, 16)
            except Exception:
                return None
        v = to_signed(v, width_bits)
        vals.append(v)
    if len(vals) % 2 != 0:
        return None
    amps = [math.sqrt(i * i + q * q) for i, q in zip(vals[0::2], vals[1::2])]
    return amps


def extract_csi_tokens(line):
    if 'CSI_BYTES:' in line:
        return line.split('CSI_BYTES:')[1].strip().split()

    if line.startswith('CSI_META,'):
        parts = line.split(',')
        if not parts:
            return []
        sample_field = parts[-1].strip().split()
        if len(sample_field) >= 2:
            return sample_field[1:]
        return []

    if line.startswith('CSI,'):
        parts = line.split(',')
        return parts[-1].strip().split()

    return []


def summarize(amps):
    if not amps:
        return "(no amps)"
    import numpy as np
    a = np.array(amps)
    return f"len={len(a)} min={a.min():.2f} max={a.max():.2f} mean={a.mean():.2f}"


def main():
    if len(sys.argv) < 2:
        print("Usage: debug_csi_dump.py <COM> [baud]")
        sys.exit(1)
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Opened {port} @ {baud}")
    count = 0
    try:
        while count < 20:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            print('\n--- LINE ---')
            print(line[:1000])
            # if CSI_BYTES: try tokens
            if 'CSI_BYTES:' in line or line.startswith('CSI,') or line.startswith('CSI_META,'):
                tokens = extract_csi_tokens(line)
                print('first tokens:', ' '.join(tokens[:20]))
                for bits in (8, 16, 32):
                    amps = amps_from_tokens(tokens, bits)
                    print(f'signed{bits}:', summarize(amps))
                # quick histogram (signed16 assumed)
                amps16 = amps_from_tokens(tokens, 16)
                if amps16:
                    from collections import Counter
                    b = [int(x) for x in amps16[:64]]
                    c = Counter((int(v) // 5) * 5 for v in b)
                    print('hist buckets (sample):', dict(list(c.items())[:10]))
            count += 1
    finally:
        ser.close()

if __name__ == '__main__':
    main()
