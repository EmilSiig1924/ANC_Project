"""
ANC microphone jig analysis script.

Compares two WAV recordings (reference + test mic) with:
- auto time-lag alignment
- band-limited RMS ratio + correlation
- raw and band-limited RMS/peak level metrics in dBFS
- optional averaged PSD (Welch-style, NumPy-only)
- optional PSD smoothing and stop/noise-floor subtraction
"""

from __future__ import annotations

import argparse
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from numpy.fft import irfft, rfft, rfftfreq


# -------------------------
# DEFAULT PARAMETERS
# -------------------------

# Derived from Data/999.9 mLH.m4a using tools/derive_band_from_audio.py
# (10-90% cumulative energy band with added margins).
DEFAULT_BAND_LOW = 30.0
DEFAULT_BAND_HIGH = 2410.0
DEFAULT_FFT_MAX_FREQ = 3000.0  # Hz
DEFAULT_MAX_LAG_MS = 250.0
DEFAULT_PSD_NPERSEG = 8192
DEFAULT_PSD_OVERLAP = 0.5
DEFAULT_SMOOTH_HZ = 20.0
DEFAULT_NOISE_ALPHA = 0.85
DEFAULT_NOISE_FLOOR_FRAC = 0.001
EPS = 1e-20


# -------------------------
# DATA STRUCTURES
# -------------------------


@dataclass
class AnalysisResult:
    fs: int
    n_samples: int
    band_low: float
    band_high: float
    ref_rms: float
    mic_rms: float
    rms_ratio: float
    corr: float
    lag_samples: int
    lag_ms: float
    ref_rms_raw: float
    mic_rms_raw: float
    ref_rms_dbfs: float
    mic_rms_dbfs: float
    ref_peak_dbfs: float
    mic_peak_dbfs: float
    ref_bp_rms_dbfs: float
    mic_bp_rms_dbfs: float


# -------------------------
# HELPERS
# -------------------------


def load_wav_mono(path: Path) -> Tuple[int, np.ndarray]:
    """
    Load WAV as mono float32 in roughly [-1, 1].
    Supports 8/16/24/32-bit PCM.
    """
    with wave.open(str(path), "rb") as wav_obj:
        fs = wav_obj.getframerate()
        n_channels = wav_obj.getnchannels()
        sampwidth = wav_obj.getsampwidth()
        n_frames = wav_obj.getnframes()
        raw = wav_obj.readframes(n_frames)

    if sampwidth == 1:
        data = np.frombuffer(raw, dtype=np.uint8).astype(np.float32)
        data = (data - 128.0) / 128.0
    elif sampwidth == 2:
        data = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
    elif sampwidth == 3:
        bytes_24 = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        data = bytes_24[:, 0] | (bytes_24[:, 1] << 8) | (bytes_24[:, 2] << 16)
        data[data >= (1 << 23)] -= 1 << 24
        data = data.astype(np.float32) / float(1 << 23)
    elif sampwidth == 4:
        data = np.frombuffer(raw, dtype=np.int32).astype(np.float32) / float(1 << 31)
    else:
        raise ValueError(f"Unsupported WAV sample width: {sampwidth} bytes")

    if n_channels > 1:
        data = data.reshape(-1, n_channels)
        data = np.mean(data, axis=1)

    return fs, data.astype(np.float32, copy=False)


def bandpass(signal: np.ndarray, fs: int, low: float, high: float, order: int = 4) -> np.ndarray:
    """
    Zero-phase FFT-domain bandpass.
    """
    del order
    nyq = 0.5 * fs
    if not (0 < low < high < nyq):
        raise ValueError(f"Band must satisfy 0 < low < high < Nyquist({nyq}). Got {low}–{high} Hz")

    n = len(signal)
    freqs = rfftfreq(n, d=1 / fs)
    spectrum = rfft(signal)
    mask = (freqs >= low) & (freqs <= high)
    filtered_spectrum = np.zeros_like(spectrum)
    filtered_spectrum[mask] = spectrum[mask]
    return irfft(filtered_spectrum, n=n).astype(np.float32, copy=False)


def rms(signal: np.ndarray) -> float:
    return float(np.sqrt(np.mean(signal ** 2)))


def amp_to_dbfs(amplitude: float) -> float:
    if amplitude <= 0.0:
        return float("-inf")
    return float(20.0 * np.log10(amplitude))


def peak_dbfs(signal: np.ndarray) -> float:
    return amp_to_dbfs(float(np.max(np.abs(signal))))


def normalized_correlation(x: np.ndarray, y: np.ndarray) -> float:
    x = x - np.mean(x)
    y = y - np.mean(y)
    denom = np.sqrt(np.dot(x, x) * np.dot(y, y))
    if denom == 0:
        return 0.0
    return float(np.dot(x, y) / denom)


def compute_fft_mag(signal: np.ndarray, fs: int, max_freq: float) -> Tuple[np.ndarray, np.ndarray]:
    n = len(signal)
    freqs = rfftfreq(n, d=1 / fs)
    mag = np.abs(rfft(signal))
    mask = freqs <= max_freq
    return freqs[mask], mag[mask]


def welch_psd(
    signal: np.ndarray,
    fs: int,
    nperseg: int = DEFAULT_PSD_NPERSEG,
    overlap: float = DEFAULT_PSD_OVERLAP,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    NumPy-only Welch PSD estimate (one-sided, linear power/Hz).
    """
    if nperseg < 64:
        raise ValueError("psd_nperseg must be >= 64")
    if not (0.0 <= overlap < 1.0):
        raise ValueError("psd_overlap must be in [0, 1)")

    step = max(1, int(round(nperseg * (1.0 - overlap))))
    window = np.hanning(nperseg).astype(np.float64)
    scale = fs * np.sum(window ** 2)

    x = signal.astype(np.float64, copy=False)
    if len(x) < nperseg:
        x = np.pad(x, (0, nperseg - len(x)), mode="constant")
        starts = [0]
    else:
        starts = list(range(0, len(x) - nperseg + 1, step))
        if not starts:
            starts = [0]

    psd_accum = np.zeros(nperseg // 2 + 1, dtype=np.float64)
    for start in starts:
        seg = x[start : start + nperseg]
        spec = np.fft.rfft(seg * window)
        p = (np.abs(spec) ** 2) / scale
        if nperseg % 2 == 0:
            if p.size > 2:
                p[1:-1] *= 2.0
        else:
            if p.size > 1:
                p[1:] *= 2.0
        psd_accum += p

    psd = psd_accum / max(len(starts), 1)
    freqs = rfftfreq(nperseg, d=1 / fs)
    return freqs, psd


def smooth_spectrum(values: np.ndarray, freqs: np.ndarray, smooth_hz: float) -> np.ndarray:
    if smooth_hz <= 0.0 or values.size < 3:
        return values
    df = float(freqs[1] - freqs[0]) if freqs.size > 1 else 0.0
    if df <= 0.0:
        return values
    bins = int(round(smooth_hz / df))
    if bins <= 1:
        return values
    if bins % 2 == 0:
        bins += 1
    kernel = np.ones(bins, dtype=np.float64) / bins
    return np.convolve(values, kernel, mode="same")


def subtract_noise_psd(
    signal_psd: np.ndarray,
    signal_freqs: np.ndarray,
    noise_psd: np.ndarray,
    noise_freqs: np.ndarray,
    alpha: float = DEFAULT_NOISE_ALPHA,
    floor_frac: float = DEFAULT_NOISE_FLOOR_FRAC,
) -> np.ndarray:
    if signal_psd.size == 0 or noise_psd.size == 0:
        return signal_psd

    if signal_freqs.shape == noise_freqs.shape and np.allclose(signal_freqs, noise_freqs):
        noise_interp = noise_psd
    else:
        noise_interp = np.interp(signal_freqs, noise_freqs, noise_psd, left=noise_psd[0], right=noise_psd[-1])

    # Soft subtraction avoids unrealistic "needle dips" from over-subtraction.
    cleaned = signal_psd - (alpha * noise_interp)
    floor_global = float(np.max(signal_psd)) * 1e-12
    floor_local = floor_frac * signal_psd
    floor = np.maximum(floor_local, max(floor_global, EPS))
    return np.maximum(cleaned, floor)


def normalize_plot_path(path: Path) -> Path:
    return path if path.suffix else path.with_suffix(".png")


def estimate_lag_samples_fft(ref: np.ndarray, mic: np.ndarray, max_lag_samples: int) -> int:
    n = len(ref)
    m = len(mic)
    if n == 0 or m == 0:
        return 0

    x = ref.astype(np.float64, copy=False) - float(np.mean(ref))
    y = mic.astype(np.float64, copy=False) - float(np.mean(mic))

    full_len = n + m - 1
    n_fft = 1
    while n_fft < full_len:
        n_fft <<= 1

    corr_circular = np.fft.ifft(np.fft.fft(x, n_fft) * np.conj(np.fft.fft(y, n_fft))).real
    if m > 1:
        corr_linear = np.concatenate([corr_circular[-(m - 1) :], corr_circular[:n]])
    else:
        corr_linear = corr_circular[:n]
    lags = np.arange(-(m - 1), n, dtype=np.int64)

    max_lag = int(min(max(0, max_lag_samples), max(abs(lags[0]), abs(lags[-1]))))
    mask = np.abs(lags) <= max_lag
    candidate_lags = lags[mask]
    candidate_corr = corr_linear[mask]
    best_idx = int(np.argmax(np.abs(candidate_corr)))
    return int(candidate_lags[best_idx])


def align_by_lag(ref: np.ndarray, mic: np.ndarray, lag_samples: int) -> Tuple[np.ndarray, np.ndarray]:
    if lag_samples > 0:
        ref_start = lag_samples
        mic_start = 0
    else:
        ref_start = 0
        mic_start = -lag_samples

    n = min(len(ref) - ref_start, len(mic) - mic_start)
    if n <= 0:
        raise ValueError("Lag alignment removed all overlap. Decrease --max-lag-ms or check recordings.")
    return ref[ref_start : ref_start + n], mic[mic_start : mic_start + n]


# -------------------------
# ANALYSIS CORE
# -------------------------


def analyze_pair(
    ref_path: Path,
    mic_path: Path,
    band_low: float,
    band_high: float,
    fft_max_freq: float,
    auto_align: bool = True,
    max_lag_ms: float = DEFAULT_MAX_LAG_MS,
    psd_nperseg: int = DEFAULT_PSD_NPERSEG,
    psd_overlap: float = DEFAULT_PSD_OVERLAP,
    smooth_hz: float = DEFAULT_SMOOTH_HZ,
    noise_wav: Optional[Path] = None,
    ref_noise_wav: Optional[Path] = None,
    mic_noise_wav: Optional[Path] = None,
    noise_alpha: float = DEFAULT_NOISE_ALPHA,
    noise_floor_frac: float = DEFAULT_NOISE_FLOOR_FRAC,
    plot: bool = False,
    plot_fft: bool = False,
    save_plot: Optional[Path] = None,
    show_after_save: bool = False,
) -> AnalysisResult:
    fs_ref, ref = load_wav_mono(ref_path)
    fs_mic, mic = load_wav_mono(mic_path)
    if fs_ref != fs_mic:
        raise ValueError(f"Sample rates differ: ref={fs_ref} Hz, mic={fs_mic} Hz")

    fs = fs_ref
    n = min(len(ref), len(mic))
    ref = ref[:n]
    mic = mic[:n]

    lag_samples = 0
    if auto_align:
        max_lag_samples = int(round((max_lag_ms / 1000.0) * fs))
        ref_bp_for_lag = bandpass(ref, fs, band_low, band_high)
        mic_bp_for_lag = bandpass(mic, fs, band_low, band_high)
        lag_samples = estimate_lag_samples_fft(ref_bp_for_lag, mic_bp_for_lag, max_lag_samples)
        ref, mic = align_by_lag(ref, mic, lag_samples)

    ref_bp = bandpass(ref, fs, band_low, band_high)
    mic_bp = bandpass(mic, fs, band_low, band_high)

    ref_rms_raw = rms(ref)
    mic_rms_raw = rms(mic)
    ref_rms = rms(ref_bp)
    mic_rms = rms(mic_bp)
    ratio = (mic_rms / ref_rms) if ref_rms != 0 else float("inf")
    corr = normalized_correlation(ref_bp, mic_bp)

    ref_rms_dbfs = amp_to_dbfs(ref_rms_raw)
    mic_rms_dbfs = amp_to_dbfs(mic_rms_raw)
    ref_peak = peak_dbfs(ref)
    mic_peak = peak_dbfs(mic)
    ref_bp_rms_dbfs = amp_to_dbfs(ref_rms)
    mic_bp_rms_dbfs = amp_to_dbfs(mic_rms)

    if plot:
        import matplotlib.pyplot as plt

        fft_fig = None
        psd_fig = None
        chosen_ref_noise = ref_noise_wav if ref_noise_wav is not None else noise_wav
        chosen_mic_noise = mic_noise_wav if mic_noise_wav is not None else noise_wav

        ref_noise = None
        mic_noise = None
        if chosen_ref_noise is not None:
            fs_n, ref_noise = load_wav_mono(chosen_ref_noise)
            if fs_n != fs:
                raise ValueError(f"Ref noise sample rate {fs_n} != signal sample rate {fs}")
        if chosen_mic_noise is not None:
            fs_n, mic_noise = load_wav_mono(chosen_mic_noise)
            if fs_n != fs:
                raise ValueError(f"Mic noise sample rate {fs_n} != signal sample rate {fs}")

        if plot_fft:
            freqs_fft, ref_mag = compute_fft_mag(ref, fs, fft_max_freq)
            _, mic_mag = compute_fft_mag(mic, fs, fft_max_freq)
            fft_fig = plt.figure()
            plt.semilogy(freqs_fft, ref_mag, label="Reference")
            plt.semilogy(freqs_fft, mic_mag, label="Test Mic")
            plt.xlabel("Frequency (Hz)")
            plt.ylabel("Magnitude")
            plt.title("Legacy FFT Magnitude (raw)")
            plt.grid(True, alpha=0.3)
            plt.legend()

        ref_f, ref_psd = welch_psd(ref, fs, nperseg=psd_nperseg, overlap=psd_overlap)
        mic_f, mic_psd = welch_psd(mic, fs, nperseg=psd_nperseg, overlap=psd_overlap)

        if ref_noise is not None:
            ref_nf, ref_noise_psd = welch_psd(ref_noise, fs, nperseg=psd_nperseg, overlap=psd_overlap)
            ref_psd = subtract_noise_psd(
                ref_psd,
                ref_f,
                ref_noise_psd,
                ref_nf,
                alpha=noise_alpha,
                floor_frac=noise_floor_frac,
            )
        if mic_noise is not None:
            mic_nf, mic_noise_psd = welch_psd(mic_noise, fs, nperseg=psd_nperseg, overlap=psd_overlap)
            mic_psd = subtract_noise_psd(
                mic_psd,
                mic_f,
                mic_noise_psd,
                mic_nf,
                alpha=noise_alpha,
                floor_frac=noise_floor_frac,
            )

        ref_psd = smooth_spectrum(ref_psd, ref_f, smooth_hz)
        mic_psd = smooth_spectrum(mic_psd, mic_f, smooth_hz)

        ref_mask = ref_f <= fft_max_freq
        mic_mask = mic_f <= fft_max_freq
        ref_db = 10.0 * np.log10(np.maximum(ref_psd[ref_mask], EPS))
        mic_db = 10.0 * np.log10(np.maximum(mic_psd[mic_mask], EPS))

        psd_fig = plt.figure()
        plt.plot(ref_f[ref_mask], ref_db, label="Reference")
        plt.plot(mic_f[mic_mask], mic_db, label="Test Mic")
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("PSD (dB/Hz)")
        title = "Averaged PSD (Welch)"
        if chosen_ref_noise is not None or chosen_mic_noise is not None:
            title += " with Noise Subtraction"
        if smooth_hz > 0.0:
            title += f", Smoothed {smooth_hz:.1f} Hz"
        plt.title(title)
        plt.grid(True, alpha=0.3)
        plt.legend()

        if save_plot is not None:
            output_path = normalize_plot_path(save_plot)
            output_path.parent.mkdir(parents=True, exist_ok=True)

            if plot_fft and fft_fig is not None:
                fft_path = output_path.with_name(f"{output_path.stem}_fft{output_path.suffix}")
                fft_fig.savefig(fft_path, dpi=150, bbox_inches="tight")
                print(f"Saved plot: {fft_path.as_posix()}")
                psd_path = output_path.with_name(f"{output_path.stem}_psd{output_path.suffix}")
            else:
                psd_path = output_path

            if psd_fig is not None:
                psd_fig.savefig(psd_path, dpi=150, bbox_inches="tight")
                print(f"Saved plot: {psd_path.as_posix()}")
            if show_after_save:
                plt.show()
            else:
                plt.close("all")
        else:
            plt.show()

    return AnalysisResult(
        fs=fs,
        n_samples=len(ref),
        band_low=band_low,
        band_high=band_high,
        ref_rms=ref_rms,
        mic_rms=mic_rms,
        rms_ratio=ratio,
        corr=corr,
        lag_samples=lag_samples,
        lag_ms=(lag_samples * 1000.0 / fs),
        ref_rms_raw=ref_rms_raw,
        mic_rms_raw=mic_rms_raw,
        ref_rms_dbfs=ref_rms_dbfs,
        mic_rms_dbfs=mic_rms_dbfs,
        ref_peak_dbfs=ref_peak,
        mic_peak_dbfs=mic_peak,
        ref_bp_rms_dbfs=ref_bp_rms_dbfs,
        mic_bp_rms_dbfs=mic_bp_rms_dbfs,
    )


def format_db(value: float) -> str:
    if np.isfinite(value):
        return f"{value:.2f} dBFS"
    return "-inf dBFS"


def format_result(res: AnalysisResult, ref_path: Path, mic_path: Path) -> str:
    duration_s = res.n_samples / res.fs
    lines = [
        "=== ANC MIC JIG TEST SUMMARY ===",
        f"Reference WAV: {ref_path.as_posix()}",
        f"Test Mic WAV:  {mic_path.as_posix()}",
        f"Sample rate:   {res.fs} Hz",
        f"Duration:      {duration_s:.2f} s",
        f"Auto lag align: {res.lag_samples:+d} samples ({res.lag_ms:+.3f} ms)",
        "",
        "Raw level metrics (after lag alignment):",
        f"Ref RMS:   {res.ref_rms_raw:.6f} ({format_db(res.ref_rms_dbfs)})",
        f"Ref peak:  {format_db(res.ref_peak_dbfs)}",
        f"Mic RMS:   {res.mic_rms_raw:.6f} ({format_db(res.mic_rms_dbfs)})",
        f"Mic peak:  {format_db(res.mic_peak_dbfs)}",
        "",
        f"Band-pass: {res.band_low:.1f} – {res.band_high:.1f} Hz",
        f"Ref RMS:   {res.ref_rms:.6f} ({format_db(res.ref_bp_rms_dbfs)})",
        f"Mic RMS:   {res.mic_rms:.6f} ({format_db(res.mic_bp_rms_dbfs)})",
        f"RMS ratio (mic/ref): {res.rms_ratio:.3f}",
        "",
        f"Normalized correlation (band-limited): {res.corr:.4f}",
        "",
        "Interpretation hints:",
        "- Peak levels near 0 dBFS indicate clipping risk; target about -12 to -6 dBFS peaks.",
        "- RMS dBFS helps keep capture levels consistent between runs.",
        "- Higher correlation usually means the jig mic captures the same content as reference.",
        "- Big RMS ratio is not automatically better; check stability + resonances.",
        "- Use --noise-wav (or per-channel noise WAVs) with --plot to reduce stop-floor clutter.",
        "- If PSD has deep narrow dips, lower --noise-alpha or raise --noise-floor-frac.",
    ]
    return "\n".join(lines)


# -------------------------
# CLI
# -------------------------


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Analyze two WAV files (reference + test mic) for ANC jig testing."
    )
    parser.add_argument("--ref", type=str, required=True, help="Path to reference mic WAV file")
    parser.add_argument("--mic", type=str, required=True, help="Path to test mic WAV file")
    parser.add_argument("--band-low", type=float, default=DEFAULT_BAND_LOW, help="Band-pass low cutoff (Hz)")
    parser.add_argument("--band-high", type=float, default=DEFAULT_BAND_HIGH, help="Band-pass high cutoff (Hz)")
    parser.add_argument("--fft-max", type=float, default=DEFAULT_FFT_MAX_FREQ, help="Max frequency to plot (Hz)")
    parser.add_argument(
        "--max-lag-ms",
        type=float,
        default=DEFAULT_MAX_LAG_MS,
        help="Max absolute lag to search during auto alignment (ms)",
    )
    parser.add_argument(
        "--no-auto-align",
        action="store_true",
        help="Disable automatic time-lag alignment before metrics",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show averaged PSD plot (Welch, smoothing, optional noise subtraction).",
    )
    parser.add_argument(
        "--plot-fft",
        action="store_true",
        help="Also show legacy FFT magnitude plot for comparison.",
    )
    parser.add_argument(
        "--save-plot",
        type=str,
        default=None,
        help="Optional output PNG path for plots. If set, plotting is enabled automatically.",
    )
    parser.add_argument(
        "--show-after-save",
        action="store_true",
        help="When used with --save-plot, also show plot windows interactively.",
    )
    parser.add_argument(
        "--psd-nperseg",
        type=int,
        default=DEFAULT_PSD_NPERSEG,
        help="Welch segment size (samples).",
    )
    parser.add_argument(
        "--psd-overlap",
        type=float,
        default=DEFAULT_PSD_OVERLAP,
        help="Welch overlap ratio in [0, 1).",
    )
    parser.add_argument(
        "--smooth-hz",
        type=float,
        default=DEFAULT_SMOOTH_HZ,
        help="PSD smoothing width in Hz (0 disables).",
    )
    parser.add_argument(
        "--noise-wav",
        type=str,
        default=None,
        help="Optional stop/noise WAV used for PSD subtraction on both channels.",
    )
    parser.add_argument(
        "--ref-noise-wav",
        type=str,
        default=None,
        help="Optional stop/noise WAV specifically for reference channel PSD subtraction.",
    )
    parser.add_argument(
        "--mic-noise-wav",
        type=str,
        default=None,
        help="Optional stop/noise WAV specifically for mic channel PSD subtraction.",
    )
    parser.add_argument(
        "--noise-alpha",
        type=float,
        default=DEFAULT_NOISE_ALPHA,
        help="Noise subtraction strength in PSD domain (0..1.5, lower = fewer subtraction dips).",
    )
    parser.add_argument(
        "--noise-floor-frac",
        type=float,
        default=DEFAULT_NOISE_FLOOR_FRAC,
        help="Minimum retained PSD floor as fraction of original PSD per bin (0..1).",
    )
    args = parser.parse_args(argv)

    ref_path = Path(args.ref)
    mic_path = Path(args.mic)
    if not ref_path.exists():
        raise FileNotFoundError(f"Reference file not found: {ref_path}")
    if not mic_path.exists():
        raise FileNotFoundError(f"Mic file not found: {mic_path}")
    if args.psd_nperseg < 64:
        raise ValueError("--psd-nperseg must be >= 64")
    if not (0.0 <= args.psd_overlap < 1.0):
        raise ValueError("--psd-overlap must be in [0, 1)")
    if not (0.0 <= args.noise_alpha <= 1.5):
        raise ValueError("--noise-alpha must be in [0, 1.5]")
    if not (0.0 <= args.noise_floor_frac <= 1.0):
        raise ValueError("--noise-floor-frac must be in [0, 1]")

    noise_wav = Path(args.noise_wav) if args.noise_wav is not None else None
    ref_noise_wav = Path(args.ref_noise_wav) if args.ref_noise_wav is not None else None
    mic_noise_wav = Path(args.mic_noise_wav) if args.mic_noise_wav is not None else None
    save_plot = Path(args.save_plot) if args.save_plot is not None else None
    for label, p in [
        ("noise", noise_wav),
        ("ref-noise", ref_noise_wav),
        ("mic-noise", mic_noise_wav),
    ]:
        if p is not None and not p.exists():
            raise FileNotFoundError(f"{label} file not found: {p}")

    should_plot = args.plot or save_plot is not None

    res = analyze_pair(
        ref_path=ref_path,
        mic_path=mic_path,
        band_low=args.band_low,
        band_high=args.band_high,
        fft_max_freq=args.fft_max,
        auto_align=not args.no_auto_align,
        max_lag_ms=args.max_lag_ms,
        psd_nperseg=args.psd_nperseg,
        psd_overlap=args.psd_overlap,
        smooth_hz=args.smooth_hz,
        noise_wav=noise_wav,
        ref_noise_wav=ref_noise_wav,
        mic_noise_wav=mic_noise_wav,
        noise_alpha=args.noise_alpha,
        noise_floor_frac=args.noise_floor_frac,
        plot=should_plot,
        plot_fft=args.plot_fft,
        save_plot=save_plot,
        show_after_save=args.show_after_save,
    )
    print(format_result(res, ref_path, mic_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
