from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import tempfile
import wave
from pathlib import Path
from typing import Tuple

import numpy as np


def load_wav_mono(path: Path) -> Tuple[int, np.ndarray]:
    with wave.open(str(path), "rb") as wav_obj:
        fs = wav_obj.getframerate()
        n_channels = wav_obj.getnchannels()
        sampwidth = wav_obj.getsampwidth()
        n_frames = wav_obj.getnframes()
        raw = wav_obj.readframes(n_frames)

    if sampwidth == 1:
        data = np.frombuffer(raw, dtype=np.uint8).astype(np.float64)
        data = (data - 128.0) / 128.0
    elif sampwidth == 2:
        data = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32768.0
    elif sampwidth == 3:
        b24 = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        data = b24[:, 0] | (b24[:, 1] << 8) | (b24[:, 2] << 16)
        data[data >= (1 << 23)] -= 1 << 24
        data = data.astype(np.float64) / float(1 << 23)
    elif sampwidth == 4:
        data = np.frombuffer(raw, dtype=np.int32).astype(np.float64) / float(1 << 31)
    else:
        raise ValueError(f"Unsupported sample width: {sampwidth} bytes")

    if n_channels > 1:
        data = data.reshape(-1, n_channels).mean(axis=1)

    return fs, data


def welch_psd(signal: np.ndarray, fs: int, nperseg: int, overlap: float) -> Tuple[np.ndarray, np.ndarray]:
    if nperseg < 64:
        raise ValueError("--nperseg must be >= 64")
    if not (0.0 <= overlap < 1.0):
        raise ValueError("--overlap must be in [0,1)")

    step = max(1, int(round(nperseg * (1.0 - overlap))))
    win = np.hanning(nperseg).astype(np.float64)
    scale = fs * np.sum(win ** 2)

    x = signal.astype(np.float64, copy=False)
    x = x - np.mean(x)

    if len(x) < nperseg:
        x = np.pad(x, (0, nperseg - len(x)))
        starts = [0]
    else:
        starts = list(range(0, len(x) - nperseg + 1, step))
        if not starts:
            starts = [0]

    psd_sum = np.zeros(nperseg // 2 + 1, dtype=np.float64)
    for s in starts:
        seg = x[s : s + nperseg]
        spec = np.fft.rfft(seg * win)
        p = (np.abs(spec) ** 2) / scale
        if nperseg % 2 == 0:
            if p.size > 2:
                p[1:-1] *= 2.0
        else:
            if p.size > 1:
                p[1:] *= 2.0
        psd_sum += p

    psd = psd_sum / max(len(starts), 1)
    freqs = np.fft.rfftfreq(nperseg, d=1 / fs)
    return freqs, psd


def convert_to_wav_if_needed(src: Path, sample_rate_hz: int) -> Tuple[Path, tempfile.TemporaryDirectory[str] | None]:
    if src.suffix.lower() == ".wav":
        return src, None

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("Input is not WAV and ffmpeg was not found on PATH.")

    tmp_dir = tempfile.TemporaryDirectory(prefix="pump_band_")
    out_wav = Path(tmp_dir.name) / f"{src.stem}_tmp.wav"
    cmd = [
        ffmpeg,
        "-y",
        "-i",
        str(src),
        "-ac",
        "1",
        "-ar",
        str(sample_rate_hz),
        "-sample_fmt",
        "s16",
        str(out_wav),
    ]
    result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"ffmpeg conversion failed:\n{result.stderr}")
    return out_wav, tmp_dir


def round_to_step(value: float, step: float) -> float:
    return round(value / step) * step


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Derive dominant frequency band from a pump recording (WAV or ffmpeg-supported input like M4A)."
        )
    )
    parser.add_argument("--input", type=Path, required=True, help="Input audio path (.wav/.m4a/etc).")
    parser.add_argument("--sample-rate-hz", type=int, default=48_000, help="Resample rate for non-WAV inputs.")
    parser.add_argument("--freq-min", type=float, default=20.0, help="Minimum frequency for cumulative band analysis.")
    parser.add_argument("--freq-max", type=float, default=5000.0, help="Maximum frequency for cumulative band analysis.")
    parser.add_argument("--q-low", type=float, default=0.10, help="Lower cumulative-energy quantile (0..1).")
    parser.add_argument("--q-high", type=float, default=0.90, help="Upper cumulative-energy quantile (0..1).")
    parser.add_argument("--low-margin-hz", type=float, default=10.0, help="Slack below q-low edge.")
    parser.add_argument("--high-margin-hz", type=float, default=300.0, help="Slack above q-high edge.")
    parser.add_argument("--round-hz", type=float, default=10.0, help="Round recommended band edges to this step.")
    parser.add_argument("--nperseg", type=int, default=8192, help="Welch segment size.")
    parser.add_argument("--overlap", type=float, default=0.5, help="Welch overlap [0,1).")
    parser.add_argument("--save-plot", type=Path, default=None, help="Optional output PNG for PSD plot.")
    args = parser.parse_args()

    if not args.input.exists():
        raise FileNotFoundError(f"Input not found: {args.input}")
    if not (0.0 < args.q_low < args.q_high < 1.0):
        raise ValueError("Need 0 < q-low < q-high < 1")
    if args.freq_min <= 0.0 or args.freq_max <= args.freq_min:
        raise ValueError("Need 0 < freq-min < freq-max")
    if args.round_hz <= 0.0:
        raise ValueError("--round-hz must be > 0")

    wav_path, tmp = convert_to_wav_if_needed(args.input, args.sample_rate_hz)
    try:
        fs, x = load_wav_mono(wav_path)
        freqs, psd = welch_psd(x, fs, nperseg=args.nperseg, overlap=args.overlap)
    finally:
        if tmp is not None:
            tmp.cleanup()

    mask = (freqs >= args.freq_min) & (freqs <= args.freq_max)
    f = freqs[mask]
    p = psd[mask]
    if f.size < 8 or np.sum(p) <= 0.0:
        raise RuntimeError("Insufficient spectral energy in selected range.")

    cdf = np.cumsum(p)
    cdf = cdf / cdf[-1]
    q_low_hz = float(np.interp(args.q_low, cdf, f))
    q_high_hz = float(np.interp(args.q_high, cdf, f))

    rec_low = max(args.freq_min, q_low_hz - args.low_margin_hz)
    rec_high = min(args.freq_max, q_high_hz + args.high_margin_hz)
    rec_low = round_to_step(rec_low, args.round_hz)
    rec_high = round_to_step(rec_high, args.round_hz)

    peak_idx = np.argsort(p)[-8:][::-1]
    print("=== Dominant Band Derivation ===")
    print(f"Input: {args.input.as_posix()}")
    print(f"Sample rate: {fs} Hz")
    print(f"Analysis range: {args.freq_min:.1f} - {args.freq_max:.1f} Hz")
    print(f"Energy quantile band: {args.q_low:.2f}-{args.q_high:.2f}")
    print(f"Quantile edges: {q_low_hz:.1f} - {q_high_hz:.1f} Hz")
    print(f"Recommended (with margins): {rec_low:.1f} - {rec_high:.1f} Hz")
    print("Top PSD peaks:")
    for i in peak_idx:
        val_db = 10.0 * math.log10(max(float(p[i]), 1e-20))
        print(f"  {f[i]:7.1f} Hz  {val_db:7.2f} dB/Hz")

    if args.save_plot is not None:
        import matplotlib.pyplot as plt

        args.save_plot.parent.mkdir(parents=True, exist_ok=True)
        db = 10.0 * np.log10(np.maximum(p, 1e-20))
        plt.figure()
        plt.plot(f, db, label="Source PSD")
        plt.axvline(q_low_hz, linestyle="--", linewidth=1.2, label=f"q_low {q_low_hz:.1f} Hz")
        plt.axvline(q_high_hz, linestyle="--", linewidth=1.2, label=f"q_high {q_high_hz:.1f} Hz")
        plt.axvline(rec_low, linestyle=":", linewidth=1.2, label=f"rec_low {rec_low:.1f} Hz")
        plt.axvline(rec_high, linestyle=":", linewidth=1.2, label=f"rec_high {rec_high:.1f} Hz")
        plt.title("Source Pump PSD (Welch)")
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("PSD (dB/Hz)")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.savefig(args.save_plot, dpi=150, bbox_inches="tight")
        plt.close("all")
        print(f"Saved plot: {args.save_plot.as_posix()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

