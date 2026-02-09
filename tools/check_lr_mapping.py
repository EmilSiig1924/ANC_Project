from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
import wave
from pathlib import Path
from typing import Tuple


def run_or_fail(cmd: list[str]) -> None:
    print("Running:", " ".join(cmd))
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise SystemExit(f"Command failed with exit code {result.returncode}")


def dbfs(amplitude: float) -> float:
    if amplitude <= 0.0:
        return float("-inf")
    return 20.0 * math.log10(amplitude)


def fmt_db(value: float) -> str:
    if math.isfinite(value):
        return f"{value:.2f} dBFS"
    return "-inf dBFS"


def run_countdown(seconds: int, message: str) -> None:
    if seconds <= 0:
        return
    print(message)
    for sec in range(seconds, 0, -1):
        print(f"  starting in {sec}s...", flush=True)
        time.sleep(1.0)


def read_slot_mapping(main_cpp_path: Path) -> Tuple[str, str]:
    """
    Reads firmware slot constants:
      kMic1Slot = MicSlot::<Left|Right>
      kMic2Slot = MicSlot::<Left|Right>
    Falls back to Left/Right defaults if parsing fails.
    """
    default_mic1 = "Left"
    default_mic2 = "Right"

    if not main_cpp_path.exists():
        return default_mic1, default_mic2

    text = main_cpp_path.read_text(encoding="utf-8", errors="ignore")
    m1 = re.search(r"kMic1Slot\s*=\s*MicSlot::(Left|Right)", text)
    m2 = re.search(r"kMic2Slot\s*=\s*MicSlot::(Left|Right)", text)
    mic1 = m1.group(1) if m1 else default_mic1
    mic2 = m2.group(1) if m2 else default_mic2
    return mic1, mic2


def analyze_stereo_wav(path: Path) -> Tuple[int, int, float, float, float, float]:
    """
    Returns:
      sample_rate_hz, frames, ch0_rms, ch1_rms, ch0_peak, ch1_peak
    where RMS/peak are normalized to full-scale (0..1).
    """
    with wave.open(str(path), "rb") as wav_in:
        channels = wav_in.getnchannels()
        sampwidth = wav_in.getsampwidth()
        sample_rate_hz = wav_in.getframerate()
        frames = wav_in.getnframes()
        raw = wav_in.readframes(frames)

    if channels != 2:
        raise SystemExit(f"Expected stereo WAV, got {channels} channels: {path}")
    if sampwidth != 2:
        raise SystemExit(f"Expected 16-bit PCM WAV, got sample width {sampwidth} bytes: {path}")
    if frames <= 0:
        raise SystemExit(f"Empty WAV: {path}")

    frame_size = 4  # 2 channels * 2 bytes
    sum_sq_0 = 0.0
    sum_sq_1 = 0.0
    peak_0 = 0
    peak_1 = 0

    for i in range(0, len(raw), frame_size):
        s0 = int.from_bytes(raw[i : i + 2], byteorder="little", signed=True)
        s1 = int.from_bytes(raw[i + 2 : i + 4], byteorder="little", signed=True)
        sum_sq_0 += float(s0 * s0)
        sum_sq_1 += float(s1 * s1)
        a0 = abs(s0)
        a1 = abs(s1)
        if a0 > peak_0:
            peak_0 = a0
        if a1 > peak_1:
            peak_1 = a1

    full_scale = 32768.0
    ch0_rms = math.sqrt(sum_sq_0 / frames) / full_scale
    ch1_rms = math.sqrt(sum_sq_1 / frames) / full_scale
    ch0_peak = peak_0 / full_scale
    ch1_peak = peak_1 / full_scale
    return sample_rate_hz, frames, ch0_rms, ch1_rms, ch0_peak, ch1_peak


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Verify L/R mapping using one simultaneous MIC=0 capture. "
            "Plays source near one side and reports which channel is louder."
        )
    )
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g., COM7).")
    parser.add_argument("--baud", type=int, default=115_200, help="Serial baud rate.")
    parser.add_argument("--duration-ms", type=int, default=3000, help="Test capture duration in ms.")
    parser.add_argument("--timeout-s", type=float, default=180.0, help="Serial timeout budget in seconds.")
    parser.add_argument("--out-dir", type=Path, default=Path("Data"), help="Directory for captured test WAV.")
    parser.add_argument(
        "--tag",
        type=str,
        default=None,
        help="Optional tag for output name. Defaults to timestamp.",
    )
    parser.add_argument(
        "--near",
        choices=["left", "right"],
        default=None,
        help="Optional side where you place the source. Used for explicit pass/fail wording.",
    )
    parser.add_argument(
        "--countdown-s",
        type=int,
        default=5,
        help="Countdown before capture starts.",
    )
    parser.add_argument("--no-countdown", action="store_true", help="Disable pre-capture countdown.")
    parser.add_argument("--debug-io", action="store_true", help="Pass --debug-io to record_usb.py.")
    parser.add_argument("--skip-preflight", action="store_true", help="Pass --skip-preflight to record_usb.py.")
    args = parser.parse_args()

    if args.duration_ms < 100:
        raise SystemExit("--duration-ms must be >= 100")
    if args.countdown_s < 0:
        raise SystemExit("--countdown-s must be >= 0")

    tag = args.tag or time.strftime("%Y%m%d_%H%M%S")
    out_path = args.out_dir / f"lrcheck_{tag}_stereo.wav"
    args.out_dir.mkdir(parents=True, exist_ok=True)

    print("=== L/R Mapping Check (simultaneous capture) ===")
    print("This uses MIC=0 (stereo) so both mics are recorded at the same time.")
    print("Guidance: keep room quiet, then play a clear source very close to only one mic.")
    if args.near is not None:
        print(f"Declared source side: {args.near.upper()}")
    print(f"Output WAV: {out_path.as_posix()}")

    if not args.no_countdown:
        run_countdown(args.countdown_s, "Prepare source position now.")

    recorder_script = Path(__file__).with_name("record_usb.py")
    if not recorder_script.exists():
        raise SystemExit(f"Missing recorder script: {recorder_script}")

    record_cmd = [
        sys.executable,
        str(recorder_script),
        "--mic",
        "0",
        "--out",
        str(out_path),
        "--duration-ms",
        str(args.duration_ms),
        "--baud",
        str(args.baud),
        "--timeout-s",
        str(args.timeout_s),
    ]
    if args.port:
        record_cmd += ["--port", args.port]
    if args.debug_io:
        record_cmd.append("--debug-io")
    if args.skip_preflight:
        record_cmd.append("--skip-preflight")

    run_or_fail(record_cmd)

    sr, frames, ch0_rms, ch1_rms, ch0_peak, ch1_peak = analyze_stereo_wav(out_path)
    ch0_rms_db = dbfs(ch0_rms)
    ch1_rms_db = dbfs(ch1_rms)
    ch0_peak_db = dbfs(ch0_peak)
    ch1_peak_db = dbfs(ch1_peak)

    mic1_slot, mic2_slot = read_slot_mapping(Path("src/main.cpp"))

    # Firmware mapping (from src/main.cpp capture packing):
    # channel 0 -> MIC=1
    # channel 1 -> MIC=2
    louder_channel = 0 if ch0_rms >= ch1_rms else 1
    quieter_channel = 1 - louder_channel
    louder_mic = 1 if louder_channel == 0 else 2
    louder_slot = mic1_slot if louder_mic == 1 else mic2_slot
    louder_db = ch0_rms_db if louder_channel == 0 else ch1_rms_db
    quieter_db = ch0_rms_db if quieter_channel == 0 else ch1_rms_db
    diff_db = louder_db - quieter_db if math.isfinite(louder_db) and math.isfinite(quieter_db) else float("inf")

    print("\n=== Result ===")
    print(f"Sample rate: {sr} Hz, frames: {frames}, duration: {frames / sr:.2f}s")
    print(
        f"ch0 -> MIC=1 ({mic1_slot} slot): RMS {fmt_db(ch0_rms_db)}, Peak {fmt_db(ch0_peak_db)}"
    )
    print(
        f"ch1 -> MIC=2 ({mic2_slot} slot): RMS {fmt_db(ch1_rms_db)}, Peak {fmt_db(ch1_peak_db)}"
    )
    if math.isfinite(diff_db):
        print(f"Loudness gap (RMS): {diff_db:.2f} dB in favor of channel {louder_channel}")
    else:
        print("Loudness gap (RMS): not finite (one channel near silence).")

    print(
        f"Dominant side in this check: MIC={louder_mic} ({louder_slot} slot)."
    )
    print("Pair-test mapping reminder:")
    print(f"- If run with --ref-mic 1: Reference={mic1_slot}, Test={mic2_slot}")
    print(f"- If run with --ref-mic 2: Reference={mic2_slot}, Test={mic1_slot}")

    if args.near is not None:
        expected = args.near.capitalize()
        if louder_slot == expected:
            print(f"Check: PASS. Your declared {expected} side matches dominant channel.")
        else:
            print(
                f"Check: MISMATCH. You declared {expected}, but dominant channel maps to {louder_slot}."
            )
            print(
                "Interpretation: your physical labeling/orientation is opposite firmware slot naming."
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
