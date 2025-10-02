#!/usr/bin/env python3
import sounddevice as sd
import numpy as np
import sys, time

# ---- Tunables ----
SAMPLE_RATE = 44100
CHANNELS = 1
FRAMES_PER_BUFFER = 256
INPUT_GAIN = 1.0         # try 0.6–1.2
NOISE_GATE_THRESH = 1e-4 # lower => opens more easily
NOISE_GATE_ATTACK = 0.2  # how fast it opens (0..1)
NOISE_GATE_RELEASE = 0.005
HPF_ALPHA = 0.995        # 1st-order high-pass; closer to 1 = lower cutoff (~120 Hz at 0.995)
LIMIT_THRESH = 0.9       # soft limiter knee
INPUT_HINT = "USB PnP"   # substring to auto-pick your USB device
OUTPUT_HINT = "USB PnP"
# -------------------

def pick_device(kind: str, hint: str):
    devs = sd.query_devices()
    matches = []
    for i, d in enumerate(devs):
        if hint.lower() in d['name'].lower():
            if kind == 'input' and d['max_input_channels'] > 0:
                matches.append(i)
            if kind == 'output' and d['max_output_channels'] > 0:
                matches.append(i)
    if not matches:
        raise ValueError(f"No {kind} device found containing '{hint}'. Use --list.")
    return matches[0]

def list_devices():
    print("\n=== Devices ===")
    for i, d in enumerate(sd.query_devices()):
        print(f"[{i}] {d['name']}  (in {d['max_input_channels']}, out {d['max_output_channels']})")
    print("Use --in N and/or --out N to override.\n")

def soft_limiter(x, th=0.9):
    # simple soft-knee: compress values past threshold
    absx = np.abs(x)
    over = absx > th
    y = np.copy(x)
    y[over] = np.sign(x[over]) * (th + (absx[over] - th) / (1.0 + (absx[over] - th) * 10.0))
    return np.clip(y, -1.0, 1.0)

def main():
    if '--list' in sys.argv:
        list_devices()
        sys.exit(0)

    # Resolve devices
    try:
        in_idx = int(sys.argv[sys.argv.index('--in') + 1]) if '--in' in sys.argv else pick_device('input', INPUT_HINT)
        out_idx = int(sys.argv[sys.argv.index('--out') + 1]) if '--out' in sys.argv else pick_device('output', OUTPUT_HINT)
    except Exception as e:
        print(f"Device selection error: {e}")
        sys.exit(1)

    print(f"Using input : [{in_idx}] {sd.query_devices(in_idx)['name']}")
    print(f"Using output: [{out_idx}] {sd.query_devices(out_idx)['name']}")
    print("Ctrl+C to stop.")

    # State for filters/gate
    prev_in = 0.0
    gate_level = 0.0

    def callback(indata, outdata, frames, time_info, status):
        nonlocal prev_in, gate_level
        if status:
            print(status, file=sys.stderr)

        x = indata[:, 0].astype(np.float32)

        # 1) High-pass (one-pole): y[n] = x[n] - x[n-1] + a*y[n-1]
        # Implement as: hp = x - prev_in + HPF_ALPHA * hp_prev (keep hp_prev in prev_in for brevity)
        # Better: keep separate state for y[n-1]; here’s correct two-state version:
        if not hasattr(callback, "hp_prev_x"):
            callback.hp_prev_x = 0.0
            callback.hp_prev_y = 0.0
        hp_y = x - callback.hp_prev_x + HPF_ALPHA * callback.hp_prev_y
        callback.hp_prev_x = x[-1]  # last sample
        callback.hp_prev_y = hp_y[-1]

        # 2) Simple level estimate for noise gate (RMS-ish)
        level = np.sqrt(np.mean(hp_y * hp_y) + 1e-12)
        target = 1.0 if level > NOISE_GATE_THRESH else 0.0
        # smooth gate control
        alpha = NOISE_GATE_ATTACK if target > gate_level else NOISE_GATE_RELEASE
        gate_level = gate_level + alpha * (target - gate_level)
        gated = hp_y * gate_level

        # 3) Gain then soft limiter
        y = gated * INPUT_GAIN
        y = soft_limiter(y, LIMIT_THRESH)

        outdata[:, 0] = y

    stream = sd.Stream(
        samplerate=SAMPLE_RATE,
        blocksize=FRAMES_PER_BUFFER,
        dtype='float32',
        channels=CHANNELS,
        callback=callback,
        device=(in_idx, out_idx),
        latency='low'
    )
    with stream:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping…")

if __name__ == "__main__":
    main()
