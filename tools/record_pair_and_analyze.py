from __future__ import annotations

"""
High-level runner for capture + analysis.

Default flow:
1) Capture simultaneous baseline (MIC=0)
2) Capture simultaneous run (MIC=0) or legacy sequential captures
3) Split/mapping into reference + test WAV files
4) Run analyze_mic_test.py and save plots
"""

import argparse
import subprocess
import sys
import time
import wave
from pathlib import Path

# -------------------------
# Process helpers
# -------------------------


def run_or_fail(cmd: list[str]) -> None:
    print("Running:", " ".join(cmd))
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise SystemExit(f"Command failed with exit code {result.returncode}")


def estimate_capture_total_seconds(duration_ms: int, baud: int, channels: int) -> float:
    capture_s = duration_ms / 1000.0
    payload_bytes = capture_s * 48_000 * 2 * channels  # 16-bit PCM, 48 kHz
    transfer_s = payload_bytes / max(baud / 10.0, 1.0)
    return capture_s + transfer_s


def stage_banner(stage_idx: int, stage_total: int, title: str) -> None:
    print(f"\n=== Stage {stage_idx}/{stage_total}: {title} ===")


def run_countdown(seconds: int, message: str) -> None:
    if seconds <= 0:
        return
    print(message)
    for sec in range(seconds, 0, -1):
        print(f"  starting in {sec}s...", flush=True)
        time.sleep(1.0)


def split_stereo_to_mics(stereo_wav: Path, mic1_wav: Path, mic2_wav: Path) -> None:
    with wave.open(str(stereo_wav), "rb") as wav_in:
        channels = wav_in.getnchannels()
        sampwidth = wav_in.getsampwidth()
        sample_rate_hz = wav_in.getframerate()
        n_frames = wav_in.getnframes()
        raw = wav_in.readframes(n_frames)

    if channels != 2:
        raise SystemExit(
            f"Expected stereo baseline from MIC=0, got {channels} channel(s): {stereo_wav}"
        )

    frame_bytes = channels * sampwidth
    ch1 = bytearray()
    ch2 = bytearray()
    for offset in range(0, len(raw), frame_bytes):
        frame = raw[offset : offset + frame_bytes]
        ch1.extend(frame[0:sampwidth])
        ch2.extend(frame[sampwidth : (2 * sampwidth)])

    for out_path, payload in [(mic1_wav, ch1), (mic2_wav, ch2)]:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with wave.open(str(out_path), "wb") as wav_out:
            wav_out.setnchannels(1)
            wav_out.setsampwidth(sampwidth)
            wav_out.setframerate(sample_rate_hz)
            wav_out.writeframes(payload)


# -------------------------
# CLI entry
# -------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Record simultaneous baseline + reference + test captures and run analyze_mic_test.py."
        )
    )
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g., COM7).")
    parser.add_argument("--baud", type=int, default=115_200, help="Serial baud rate.")
    parser.add_argument("--duration-ms", type=int, default=20_000, help="Capture duration in milliseconds.")
    parser.add_argument(
        "--baseline-ms",
        type=int,
        default=3000,
        help="Stop-condition baseline duration in milliseconds (captured with MIC=0 stereo). Use 0 to disable.",
    )
    parser.add_argument("--timeout-s", type=float, default=300.0, help="Per-capture timeout budget in seconds.")
    parser.add_argument("--out-dir", type=Path, default=Path("Data"), help="Output directory for WAV files.")
    parser.add_argument("--plots-dir", type=Path, default=Path("Plots"), help="Default directory for saved plot PNGs.")
    parser.add_argument("--prefix", type=str, default="pair", help="Filename prefix.")
    parser.add_argument(
        "--tag",
        type=str,
        default=None,
        help="Optional run tag used in output names. Defaults to timestamp.",
    )
    parser.add_argument(
        "--ref-mic",
        type=int,
        choices=[1, 2],
        default=2,
        help="Mic index to treat as reference for analysis.",
    )
    parser.add_argument("--analyze-script", type=Path, default=Path("analyze_mic_test.py"), help="Analysis script path.")
    parser.add_argument("--band-low", type=float, default=None, help="Optional analyze_mic_test.py --band-low override.")
    parser.add_argument("--band-high", type=float, default=None, help="Optional analyze_mic_test.py --band-high override.")
    parser.add_argument(
        "--sequential-run",
        action="store_true",
        help="Capture ref/test in two separate mono recordings (legacy mode). Default is simultaneous stereo run capture.",
    )
    parser.add_argument(
        "--noise-alpha",
        type=float,
        default=None,
        help="Optional analyze_mic_test.py --noise-alpha override (lower values reduce deep subtraction dips).",
    )
    parser.add_argument(
        "--noise-floor-frac",
        type=float,
        default=None,
        help="Optional analyze_mic_test.py --noise-floor-frac override.",
    )
    parser.add_argument(
        "--countdown-s",
        type=int,
        default=5,
        help="Countdown seconds before each capture stage.",
    )
    parser.add_argument(
        "--no-countdown",
        action="store_true",
        help="Disable capture countdown prompts.",
    )
    parser.add_argument("--no-plot", action="store_true", help="Disable plotting during analysis.")
    fft_group = parser.add_mutually_exclusive_group()
    fft_group.add_argument(
        "--plot-fft",
        dest="plot_fft",
        action="store_true",
        help="Include legacy FFT plot alongside PSD (default: enabled).",
    )
    fft_group.add_argument(
        "--no-plot-fft",
        dest="plot_fft",
        action="store_false",
        help="Disable legacy FFT plot and keep PSD-only plotting.",
    )
    parser.set_defaults(plot_fft=True)
    parser.add_argument(
        "--save-plot",
        type=Path,
        default=None,
        help="Optional explicit plot PNG path. Defaults to Plots/<prefix>_<tag>.png.",
    )
    parser.add_argument("--no-save-plot", action="store_true", help="Disable saving plots to disk.")
    parser.add_argument(
        "--no-show-plot",
        action="store_true",
        help="Do not show plot windows after saving.",
    )
    parser.add_argument("--debug-io", action="store_true", help="Pass --debug-io to record_usb.py.")
    parser.add_argument("--skip-preflight", action="store_true", help="Pass --skip-preflight to record_usb.py.")
    args = parser.parse_args()
    if args.baseline_ms < 0:
        raise SystemExit("--baseline-ms must be >= 0")
    if 0 < args.baseline_ms < 100:
        raise SystemExit("--baseline-ms must be 0 or >= 100 (firmware minimum capture length).")
    if args.countdown_s < 0:
        raise SystemExit("--countdown-s must be >= 0")

    tag = args.tag or time.strftime("%Y%m%d_%H%M%S")
    ref_mic = args.ref_mic
    test_mic = 2 if ref_mic == 1 else 1
    simultaneous_run = not args.sequential_run

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    baseline_stereo_wav = out_dir / f"{args.prefix}_{tag}_baseline_stereo.wav"
    baseline_mic1_wav = out_dir / f"{args.prefix}_{tag}_baseline_mic1.wav"
    baseline_mic2_wav = out_dir / f"{args.prefix}_{tag}_baseline_mic2.wav"
    run_stereo_wav = out_dir / f"{args.prefix}_{tag}_run_stereo.wav"
    ref_wav = out_dir / f"{args.prefix}_{tag}_ref_mic{ref_mic}.wav"
    test_wav = out_dir / f"{args.prefix}_{tag}_test_mic{test_mic}.wav"
    default_plot_path = args.plots_dir / f"{args.prefix}_{tag}.png"
    plot_path = args.save_plot if args.save_plot is not None else default_plot_path

    recorder_script = Path(__file__).with_name("record_usb.py")
    if not recorder_script.exists():
        raise SystemExit(f"Missing recorder script: {recorder_script}")
    if not args.analyze_script.exists():
        raise SystemExit(f"Missing analysis script: {args.analyze_script}")

    common_record_args = [
        "--baud",
        str(args.baud),
        "--duration-ms",
        str(args.duration_ms),
        "--timeout-s",
        str(args.timeout_s),
    ]
    if args.port:
        common_record_args += ["--port", args.port]
    if args.debug_io:
        common_record_args.append("--debug-io")
    if args.skip_preflight:
        common_record_args.append("--skip-preflight")

    print(f"Run tag: {tag}")
    if args.baseline_ms > 0:
        print(f"Baseline stereo: {baseline_stereo_wav.as_posix()}")
        print(f"Baseline mic1:   {baseline_mic1_wav.as_posix()}")
        print(f"Baseline mic2:   {baseline_mic2_wav.as_posix()}")
    if simultaneous_run:
        print(f"Run stereo:      {run_stereo_wav.as_posix()}")
    print(f"Reference file: {ref_wav.as_posix()}")
    print(f"Test file:      {test_wav.as_posix()}")
    if not args.no_save_plot:
        print(f"Plot file:      {plot_path.as_posix()}")

    if simultaneous_run:
        stage_total = 3 if args.baseline_ms > 0 else 2
    else:
        stage_total = 4 if args.baseline_ms > 0 else 3
    stage_idx = 1

    if args.baseline_ms > 0:
        stage_banner(stage_idx, stage_total, "Baseline STOP capture (MIC=0, stereo)")
        print("Action: set pump to STOP and keep environment steady.")
        print(f"This baseline will run for {args.baseline_ms / 1000.0:.1f}s.")
        if not args.no_countdown:
            run_countdown(args.countdown_s, "Prepare now for STOP baseline capture.")
        baseline_expected_s = estimate_capture_total_seconds(args.baseline_ms, args.baud, channels=2)
        print(f"Expected baseline capture+transfer: ~{baseline_expected_s:.1f}s")
        run_or_fail(
            [
                sys.executable,
                str(recorder_script),
                "--mic",
                "0",
                "--out",
                str(baseline_stereo_wav),
                "--duration-ms",
                str(args.baseline_ms),
                "--baud",
                str(args.baud),
                "--timeout-s",
                str(args.timeout_s),
                *(["--port", args.port] if args.port else []),
                *(["--debug-io"] if args.debug_io else []),
                *(["--skip-preflight"] if args.skip_preflight else []),
            ]
        )
        print("Baseline capture complete. Splitting stereo baseline into per-mic files...")
        split_stereo_to_mics(baseline_stereo_wav, baseline_mic1_wav, baseline_mic2_wav)
        print("Split baseline stereo into per-mic mono stop-condition files.")
        stage_idx += 1

    if simultaneous_run:
        stage_banner(stage_idx, stage_total, "Simultaneous RUN capture (MIC=0, stereo)")
        print("Action: set pump to RUN test condition and hold it steady.")
        print("This records reference and test mics at the exact same time.")
        print(f"Run capture duration: {args.duration_ms / 1000.0:.1f}s")
        if not args.no_countdown:
            run_countdown(args.countdown_s, "Prepare now for simultaneous RUN capture.")
        run_expected_s = estimate_capture_total_seconds(args.duration_ms, args.baud, channels=2)
        print(f"Expected run capture+transfer: ~{run_expected_s:.1f}s")
        run_or_fail(
            [
                sys.executable,
                str(recorder_script),
                "--mic",
                "0",
                "--out",
                str(run_stereo_wav),
                "--duration-ms",
                str(args.duration_ms),
                "--baud",
                str(args.baud),
                "--timeout-s",
                str(args.timeout_s),
                *(["--port", args.port] if args.port else []),
                *(["--debug-io"] if args.debug_io else []),
                *(["--skip-preflight"] if args.skip_preflight else []),
            ]
        )
        print("Run capture complete. Splitting stereo run into per-mic files...")
        # Split directly into ref/test targets to keep output file set simple.
        if ref_mic == 1:
            split_stereo_to_mics(run_stereo_wav, ref_wav, test_wav)
        else:
            split_stereo_to_mics(run_stereo_wav, test_wav, ref_wav)
        print("Mapped simultaneous capture to reference/test WAV files.")
        stage_idx += 1
    else:
        stage_banner(stage_idx, stage_total, f"Reference capture (mic {ref_mic})")
        print("Action: set pump to RUN test condition and hold it steady.")
        print(f"Reference capture duration: {args.duration_ms / 1000.0:.1f}s")
        if not args.no_countdown:
            run_countdown(args.countdown_s, "Prepare now for REFERENCE capture.")
        reference_expected_s = estimate_capture_total_seconds(args.duration_ms, args.baud, channels=1)
        print(f"Expected reference capture+transfer: ~{reference_expected_s:.1f}s")
        run_or_fail(
            [
                sys.executable,
                str(recorder_script),
                "--mic",
                str(ref_mic),
                "--out",
                str(ref_wav),
                *common_record_args,
            ]
        )
        print("Reference capture complete.")
        stage_idx += 1

        stage_banner(stage_idx, stage_total, f"Test capture (mic {test_mic})")
        print("Action: keep the same pump RUN condition for test-mic capture.")
        print(f"Test capture duration: {args.duration_ms / 1000.0:.1f}s")
        if not args.no_countdown:
            run_countdown(args.countdown_s, "Prepare now for TEST capture.")
        test_expected_s = estimate_capture_total_seconds(args.duration_ms, args.baud, channels=1)
        print(f"Expected test capture+transfer: ~{test_expected_s:.1f}s")
        run_or_fail(
            [
                sys.executable,
                str(recorder_script),
                "--mic",
                str(test_mic),
                "--out",
                str(test_wav),
                *common_record_args,
            ]
        )
        print("Test capture complete.")
        stage_idx += 1

    stage_banner(stage_idx, stage_total, "Analyze + Plot")
    print("Action: running lag alignment, RMS/peak dBFS metrics, and spectral analysis.")
    if args.baseline_ms > 0:
        print("Using per-mic STOP baseline subtraction for noise-floor cleanup.")
    if not args.no_save_plot:
        print(f"Plot will be saved to: {plot_path.as_posix()}")
    if not args.no_plot:
        if args.no_show_plot:
            print("Plot display: disabled by --no-show-plot")
        else:
            print("Plot display: window will open after save.")
    analyze_cmd = [
        sys.executable,
        str(args.analyze_script),
        "--ref",
        str(ref_wav),
        "--mic",
        str(test_wav),
    ]
    if args.band_low is not None:
        analyze_cmd += ["--band-low", str(args.band_low)]
    if args.band_high is not None:
        analyze_cmd += ["--band-high", str(args.band_high)]
    if args.noise_alpha is not None:
        analyze_cmd += ["--noise-alpha", str(args.noise_alpha)]
    if args.noise_floor_frac is not None:
        analyze_cmd += ["--noise-floor-frac", str(args.noise_floor_frac)]
    if args.baseline_ms > 0:
        ref_noise = baseline_mic1_wav if ref_mic == 1 else baseline_mic2_wav
        mic_noise = baseline_mic2_wav if test_mic == 2 else baseline_mic1_wav
        analyze_cmd += ["--ref-noise-wav", str(ref_noise), "--mic-noise-wav", str(mic_noise)]
    if not args.no_plot:
        analyze_cmd.append("--plot")
    if args.plot_fft:
        analyze_cmd.append("--plot-fft")
    if not args.no_save_plot:
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        analyze_cmd += ["--save-plot", str(plot_path)]
    if not args.no_show_plot and not args.no_plot and not args.no_save_plot:
        analyze_cmd.append("--show-after-save")
    run_or_fail(analyze_cmd)

    print("\nCompleted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
