#!/usr/bin/env python3
"""Convert FNIRSI DPOS350P waveform dumps (.wav) to CSV.

The DPOS350P "Save wave" files are not RIFF PCM WAV. This tool extracts
the raw sample block and writes a CSV with sample index and normalized value.

Usage:
  python tools/dpos350p_wave_to_csv.py Data/Waveform/7.wav
  python tools/dpos350p_wave_to_csv.py Data/Waveform/*.wav --time-div-ms 5
"""

from __future__ import annotations

import argparse
import array
import csv
import glob
import os
import struct
import sys
from pathlib import Path
from typing import Iterable


def read_u32be(blob: bytes, offset: int) -> int:
    return struct.unpack(">I", blob[offset : offset + 4])[0]


def extract_samples(blob: bytes) -> tuple[array.array, dict[str, int]]:
    if len(blob) < 24:
        raise ValueError("File too small to contain DPOS350P header.")

    header_bytes = read_u32be(blob, 0)
    trailer_bytes = read_u32be(blob, 8)
    data_offset = read_u32be(blob, 12)
    data_offset_plus_trailer = read_u32be(blob, 16)
    data_bytes = read_u32be(blob, 20)

    if data_offset + data_bytes > len(blob):
        raise ValueError("Data block points outside file.")

    data = blob[data_offset : data_offset + data_bytes]
    if len(data) < 2:
        raise ValueError("No sample data found.")

    # Samples are 16-bit little-endian signed values.
    sample_count = len(data) // 2
    data = data[: sample_count * 2]
    samples = array.array("h")
    samples.frombytes(data)
    if sys.byteorder != "little":
        samples.byteswap()

    meta = {
        "header_bytes": header_bytes,
        "trailer_bytes": trailer_bytes,
        "data_offset": data_offset,
        "data_offset_plus_trailer": data_offset_plus_trailer,
        "data_bytes": data_bytes,
        "sample_count": sample_count,
        "file_bytes": len(blob),
    }
    return samples, meta


def write_csv(
    out_path: Path,
    samples: array.array,
    time_div_ms: float | None,
) -> None:
    dt_s = None
    if time_div_ms is not None and len(samples) > 0:
        # Scope horizontal axis is 10 divisions.
        dt_s = (time_div_ms * 1e-3 * 10.0) / float(len(samples))

    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if dt_s is None:
            writer.writerow(["sample_index", "adc_counts", "norm_fs"])
            for i, s in enumerate(samples):
                writer.writerow([i, int(s), float(s) / 32768.0])
        else:
            writer.writerow(["sample_index", "time_s", "adc_counts", "norm_fs"])
            for i, s in enumerate(samples):
                writer.writerow([i, i * dt_s, int(s), float(s) / 32768.0])


def expand_inputs(patterns: Iterable[str]) -> list[Path]:
    out: list[Path] = []
    for p in patterns:
        matches = sorted(glob.glob(p))
        if not matches:
            out.append(Path(p))
        else:
            out.extend(Path(m) for m in matches)
    # Preserve order, dedupe
    unique: list[Path] = []
    seen = set()
    for p in out:
        rp = str(p.resolve()) if p.exists() else str(p)
        if rp in seen:
            continue
        seen.add(rp)
        unique.append(p)
    return unique


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="Input .wav files or glob patterns.")
    parser.add_argument(
        "--time-div-ms",
        type=float,
        default=None,
        help="Optional scope timebase in ms/div to add a time_s column.",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Optional output directory. Defaults to source file directory.",
    )
    args = parser.parse_args()

    paths = expand_inputs(args.inputs)
    if not paths:
        print("No input files found.")
        return 1

    out_dir = Path(args.out_dir) if args.out_dir else None
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)

    rc = 0
    for p in paths:
        if not p.exists():
            print(f"[ERR] Missing file: {p}")
            rc = 1
            continue
        try:
            blob = p.read_bytes()
            samples, meta = extract_samples(blob)
            target_dir = out_dir if out_dir is not None else p.parent
            out_path = target_dir / f"{p.stem}.csv"
            write_csv(out_path, samples, args.time_div_ms)
            print(
                f"[OK] {p.name} -> {out_path}  samples={meta['sample_count']} "
                f"data_offset={meta['data_offset']} data_bytes={meta['data_bytes']}"
            )
        except Exception as exc:
            print(f"[ERR] {p}: {exc}")
            rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

