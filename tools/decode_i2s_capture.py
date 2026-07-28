#!/usr/bin/env python3
"""Decode I2S/PCM words from a logic-analyzer CSV export.

Default mapping matches the current ANC/DAC wiring:
  Channel 0 = BCLK / clock
  Channel 1 = WS / LRCK / frame
  Channel 2 = serial DATA

Examples:
  python tools/decode_i2s_capture.py
  python tools/decode_i2s_capture.py Data/Logic
  python tools/decode_i2s_capture.py Data/Logic/capture.csv
  python tools/decode_i2s_capture.py Data/Logic/capture.csv --edge falling --i2s-delay 1
  python tools/decode_i2s_capture.py Data/Logic/capture.kvdat
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_CLOCK = "Channel 0"
DEFAULT_WS = "Channel 1"
DEFAULT_DATA = "Channel 2"


@dataclass(frozen=True)
class DecodedWord:
    index: int
    start_time_s: float
    end_time_s: float
    ws: int
    label: str
    value: int
    signed: int
    bit_count: int


@dataclass(frozen=True)
class EdgeStats:
    rows: int
    start_time_s: float | None
    end_time_s: float | None
    clock_edges: int
    ws_edges: int
    data_edges: int
    clock_frequency_hz: float | None
    ws_frequency_hz: float | None


def normalize_header(name: str) -> str:
    return name.strip()


def parse_bit(value: str, column: str, row_number: int) -> int:
    text = value.strip()
    if text not in {"0", "1"}:
        raise ValueError(f"Row {row_number}: {column!r} is {value!r}, expected 0 or 1.")
    return int(text)


def twos_complement(value: int, bits: int) -> int:
    sign = 1 << (bits - 1)
    return value - (1 << bits) if value & sign else value


def estimate_frequency(edge_times: list[float], edges_per_period: int) -> float | None:
    if len(edge_times) < edges_per_period + 1:
        return None

    periods: list[float] = []
    for i in range(edges_per_period, len(edge_times)):
        dt = edge_times[i] - edge_times[i - edges_per_period]
        if dt > 0:
            periods.append(dt)
    if not periods:
        return None

    median_period = statistics.median(periods)
    if median_period <= 0:
        return None
    return 1.0 / median_period


def read_csv_rows(path: Path) -> Iterable[tuple[int, float, dict[str, int]]]:
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError("CSV has no header row.")

        names = {normalize_header(name): name for name in reader.fieldnames}
        if "Time[s]" not in names:
            raise ValueError("CSV must contain a Time[s] column.")

        channel_names = [name for name in names if name.startswith("Channel ")]
        if not channel_names:
            raise ValueError("CSV must contain logic channel columns named like 'Channel 0'.")

        for row_number, row in enumerate(reader, start=2):
            time_s = float(row[names["Time[s]"]].strip())
            state = {
                channel: parse_bit(row[names[channel]], channel, row_number)
                for channel in channel_names
            }
            yield row_number, time_s, state


def analyze_edges(
    path: Path,
    clock_column: str,
    ws_column: str,
    data_column: str,
) -> EdgeStats:
    rows = 0
    start_time: float | None = None
    end_time: float | None = None
    prev_state: dict[str, int] | None = None
    clock_edge_times: list[float] = []
    ws_edge_times: list[float] = []
    data_edges = 0

    for _, time_s, state in read_csv_rows(path):
        rows += 1
        if start_time is None:
            start_time = time_s
        end_time = time_s

        if clock_column not in state or ws_column not in state or data_column not in state:
            available = ", ".join(sorted(state))
            raise ValueError(
                f"Missing one of {clock_column!r}, {ws_column!r}, {data_column!r}. "
                f"Available channels: {available}"
            )

        if prev_state is not None:
            if state[clock_column] != prev_state[clock_column]:
                clock_edge_times.append(time_s)
            if state[ws_column] != prev_state[ws_column]:
                ws_edge_times.append(time_s)
            if state[data_column] != prev_state[data_column]:
                data_edges += 1
        prev_state = state

    return EdgeStats(
        rows=rows,
        start_time_s=start_time,
        end_time_s=end_time,
        clock_edges=len(clock_edge_times),
        ws_edges=len(ws_edge_times),
        data_edges=data_edges,
        clock_frequency_hz=estimate_frequency(clock_edge_times, edges_per_period=2),
        ws_frequency_hz=estimate_frequency(ws_edge_times, edges_per_period=2),
    )


def decode_i2s_csv(
    path: Path,
    clock_column: str,
    ws_column: str,
    data_column: str,
    bits: int,
    edge: str,
    sample_state: str,
    i2s_delay: int,
    ws_low_label: str,
    ws_high_label: str,
    max_words: int | None,
    emit_partial: bool,
) -> tuple[list[DecodedWord], EdgeStats]:
    if bits <= 0:
        raise ValueError("--bits must be positive.")
    if i2s_delay < 0:
        raise ValueError("--i2s-delay must be zero or greater.")

    stats = analyze_edges(path, clock_column, ws_column, data_column)
    wanted_transition = (0, 1) if edge == "rising" else (1, 0)

    prev_state: dict[str, int] | None = None
    current_ws: int | None = None
    ws_delay_line: list[int] = []
    collecting = False
    ignore_until_ws_change = False
    word_bits: list[int] = []
    word_start_time: float | None = None
    decoded: list[DecodedWord] = []

    def label_for(ws: int) -> str:
        return ws_high_label if ws else ws_low_label

    def finish_word(end_time_s: float, partial: bool = False) -> None:
        nonlocal word_bits, word_start_time, collecting, ignore_until_ws_change

        if not word_bits:
            collecting = False
            return
        if partial and not emit_partial:
            word_bits = []
            word_start_time = None
            collecting = False
            return

        value = 0
        for bit in word_bits:
            value = (value << 1) | bit

        bit_count = len(word_bits)
        if bit_count < bits:
            value <<= bits - bit_count

        if current_ws is None or word_start_time is None:
            raise RuntimeError("Decoder internal state is inconsistent.")

        decoded.append(
            DecodedWord(
                index=len(decoded),
                start_time_s=word_start_time,
                end_time_s=end_time_s,
                ws=current_ws,
                label=label_for(current_ws),
                value=value,
                signed=twos_complement(value, bits),
                bit_count=bit_count,
            )
        )

        word_bits = []
        word_start_time = None
        collecting = False
        ignore_until_ws_change = True

    for _, time_s, state in read_csv_rows(path):
        if prev_state is None:
            prev_state = state
            current_ws = state[ws_column]
            continue

        if (prev_state[clock_column], state[clock_column]) != wanted_transition:
            prev_state = state
            continue

        sampled_state = prev_state if sample_state == "before" else state
        sampled_ws = sampled_state[ws_column]
        sampled_data = sampled_state[data_column]

        if i2s_delay > 0:
            if len(ws_delay_line) < i2s_delay:
                ws_delay_line.append(sampled_ws)
                prev_state = state
                continue
            bit_ws = ws_delay_line.pop(0)
            ws_delay_line.append(sampled_ws)
        else:
            bit_ws = sampled_ws

        if current_ws is None or bit_ws != current_ws:
            if collecting:
                finish_word(time_s, partial=True)
            current_ws = bit_ws
            ignore_until_ws_change = False
            word_bits = []
            word_start_time = None
            collecting = False

        if ignore_until_ws_change:
            prev_state = state
            continue

        if word_start_time is None:
            word_start_time = time_s
            collecting = True
        word_bits.append(sampled_data)

        if len(word_bits) == bits:
            finish_word(time_s, partial=False)
            if max_words is not None and len(decoded) >= max_words:
                break

        prev_state = state

    if collecting and (max_words is None or len(decoded) < max_words):
        finish_word(stats.end_time_s if stats.end_time_s is not None else 0.0, partial=True)

    return decoded, stats


def write_decoded_csv(path: Path, words: list[DecodedWord], summary: str | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        if summary:
            for line in summary.splitlines():
                f.write(f"# {line}\n")
            f.write("#\n")
        writer = csv.writer(f)
        writer.writerow(
            [
                "index",
                "start_time_s",
                "end_time_s",
                "ws",
                "label",
                "value_hex",
                "unsigned",
                "signed",
                "bit_count",
            ]
        )
        for word in words:
            writer.writerow(
                [
                    word.index,
                    f"{word.start_time_s:.9f}",
                    f"{word.end_time_s:.9f}",
                    word.ws,
                    word.label,
                    f"0x{word.value:04X}",
                    word.value,
                    word.signed,
                    word.bit_count,
                ]
            )


def rms(values: Iterable[int]) -> float:
    values = list(values)
    if not values:
        return 0.0
    return math.sqrt(sum(float(v) * float(v) for v in values) / len(values))


def summarize_adjacent_pairs(words: list[DecodedWord]) -> str | None:
    pairs: list[tuple[DecodedWord, DecodedWord]] = []
    i = 0
    while i + 1 < len(words):
        first = words[i]
        second = words[i + 1]
        if first.label != second.label:
            pairs.append((first, second))
            i += 2
        else:
            i += 1

    if not pairs:
        return None

    deltas = [abs(first.signed - second.signed) for first, second in pairs]
    exact = sum(1 for delta in deltas if delta == 0)
    return (
        f"Adjacent stereo pairs: frames={len(pairs)}, exact_matches={exact}, "
        f"max_abs_delta={max(deltas)}, mean_abs_delta={statistics.mean(deltas):.1f}"
    )


def summarize_words(words: list[DecodedWord], preview_count: int) -> str:
    if not words:
        return "Decoded words: 0"

    lines = [f"Decoded words: {len(words)}"]
    labels = sorted({word.label for word in words})
    for label in labels:
        subset = [word for word in words if word.label == label]
        signed_values = [word.signed for word in subset]
        unsigned_values = [word.value for word in subset]
        lines.append(
            f"  {label}: count={len(subset)}, "
            f"unsigned=[0x{min(unsigned_values):04X}..0x{max(unsigned_values):04X}], "
            f"signed=[{min(signed_values)}..{max(signed_values)}], "
            f"rms={rms(signed_values):.1f}"
        )

    pair_summary = summarize_adjacent_pairs(words)
    if pair_summary is not None:
        lines.append(pair_summary)

    lines.append("First words:")
    for word in words[:preview_count]:
        lines.append(
            f"  #{word.index:04d} {word.label:>5} ws={word.ws} "
            f"{word.value:#06x} signed={word.signed:6d} "
            f"bits={word.bit_count:2d} t={word.start_time_s:.9f}s"
        )
    return "\n".join(lines)


def format_frequency(value: float | None) -> str:
    if value is None:
        return "unknown"
    if value >= 1_000_000:
        return f"{value / 1_000_000:.6g} MHz"
    if value >= 1_000:
        return f"{value / 1_000:.6g} kHz"
    return f"{value:.6g} Hz"


def summarize_edges(stats: EdgeStats) -> str:
    if stats.start_time_s is None or stats.end_time_s is None:
        duration = "unknown"
    else:
        duration = f"{stats.end_time_s - stats.start_time_s:.9f}s"

    return "\n".join(
        [
            f"Rows: {stats.rows}",
            f"Duration: {duration}",
            f"Clock edges: {stats.clock_edges} ({format_frequency(stats.clock_frequency_hz)})",
            f"WS edges: {stats.ws_edges} ({format_frequency(stats.ws_frequency_hz)})",
            f"DATA edges: {stats.data_edges}",
        ]
    )


def extract_kvdat_xml(path: Path) -> ET.Element:
    blob = path.read_bytes()
    close_tag = b"</settings>"
    end = blob.find(close_tag)
    if end < 0:
        raise ValueError("Could not find </settings> XML block in .kvdat file.")

    xml_blob = blob[: end + len(close_tag)]
    text = xml_blob.decode("utf-8", errors="replace")
    return ET.fromstring(text)


def text_at(root: ET.Element, path: str) -> str | None:
    node = root.find(path)
    if node is None or node.text is None:
        return None
    return node.text.strip()


def summarize_kvdat(path: Path) -> str:
    root = extract_kvdat_xml(path)
    lines = [f"KVDAT metadata: {path}"]

    for label, xml_path in [
        ("software version", "./global/version"),
        ("device model", "./global/devModel"),
        ("sample depth", "./devices/LA5016/smpDepth"),
        ("sample frequency", "./devices/LA5016/smpFrequ"),
        ("enabled channels", "./devices/LA5016/chnEnable"),
    ]:
        value = text_at(root, xml_path)
        if value:
            lines.append(f"  {label}: {value}")

    for index, analyzer in enumerate(root.findall("./analyzers/*"), start=1):
        name = text_at(analyzer, "./fileName") or "unknown"
        params = text_at(analyzer, "./parameters") or ""
        lines.append(f"  analyzer {index}: {name}")
        if params:
            lines.append(f"    parameters: {params}")

    lines.append(
        "  note: this script reads .kvdat metadata, but decodes signal data from CSV exports."
    )
    return "\n".join(lines)


def default_output_path(input_path: Path) -> Path:
    return input_path.parent / "decoded" / f"{input_path.stem}_decoded_i2s.csv"


def is_decoded_csv(path: Path) -> bool:
    parts = {part.lower() for part in path.parts}
    return "decoded" in parts or path.stem.endswith("_decoded_i2s")


def iter_csv_inputs(root: Path, recursive: bool) -> Iterable[Path]:
    pattern = "**/*.csv" if recursive else "*.csv"
    for path in sorted(root.glob(pattern)):
        if path.is_file() and not is_decoded_csv(path):
            yield path


def build_summary(input_path: Path, args: argparse.Namespace, words: list[DecodedWord], stats: EdgeStats) -> str:
    return "\n".join(
        [
            f"Input: {input_path}",
            (
                f"Settings: bits={args.bits}, edge={args.edge}, "
                f"sample_state={args.sample_state}, i2s_delay={args.i2s_delay}"
            ),
            summarize_edges(stats),
            summarize_words(words, args.preview),
        ]
    )


def decode_csv_file(
    input_path: Path,
    args: argparse.Namespace,
    output_path: Path | None,
) -> None:
    words, stats = decode_i2s_csv(
        path=input_path,
        clock_column=args.clock,
        ws_column=args.ws,
        data_column=args.data,
        bits=args.bits,
        edge=args.edge,
        sample_state=args.sample_state,
        i2s_delay=args.i2s_delay,
        ws_low_label=args.ws_low_label,
        ws_high_label=args.ws_high_label,
        max_words=args.max_words,
        emit_partial=args.emit_partial,
    )

    summary = build_summary(input_path, args, words, stats)
    print(summary)

    if output_path is not None:
        write_decoded_csv(output_path, words, summary)
        print(f"Decoded CSV: {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode I2S words from a logic-analyzer CSV export."
    )
    parser.add_argument(
        "input",
        type=Path,
        nargs="?",
        default=Path("Data/Logic"),
        help="Logic analyzer .csv/.kvdat file, or a folder of raw CSVs. Default: Data/Logic",
    )
    parser.add_argument("--clock", default=DEFAULT_CLOCK, help=f"Clock column. Default: {DEFAULT_CLOCK}")
    parser.add_argument("--ws", default=DEFAULT_WS, help=f"WS/LRCK column. Default: {DEFAULT_WS}")
    parser.add_argument("--data", default=DEFAULT_DATA, help=f"DATA column. Default: {DEFAULT_DATA}")
    parser.add_argument("--bits", type=int, default=16, help="Word width in bits. Default: 16")
    parser.add_argument(
        "--edge",
        choices=("rising", "falling"),
        default="rising",
        help="BCLK edge used to sample DATA. Default: rising",
    )
    parser.add_argument(
        "--sample-state",
        choices=("after", "before"),
        default="after",
        help="Use signal states just after or just before each exported BCLK edge. Default: after",
    )
    parser.add_argument(
        "--i2s-delay",
        type=int,
        default=1,
        help="WS-to-DATA delay in bit clocks. Standard I2S is 1; left-justified/PCM is 0. Default: 1",
    )
    parser.add_argument("--ws-low-label", default="left", help="Label for WS=0 words.")
    parser.add_argument("--ws-high-label", default="right", help="Label for WS=1 words.")
    parser.add_argument(
        "--max-words",
        type=int,
        default=None,
        help="Stop after this many decoded words. Default: decode all words.",
    )
    parser.add_argument(
        "--preview",
        type=int,
        default=16,
        help="Number of decoded words to print. Default: 16",
    )
    parser.add_argument(
        "--emit-partial",
        action="store_true",
        help="Emit short words at WS transitions/end of file for debugging.",
    )
    parser.add_argument(
        "--no-output",
        action="store_true",
        help="Print summary only; do not write decoded CSV.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="When input is a folder, regenerate decoded CSVs even if they already exist.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="When input is a folder, search subfolders too. The decoded folder is ignored.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Decoded CSV output path. Default: Data/Logic/decoded/<input>_decoded_i2s.csv",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = args.input
    if not input_path.exists():
        raise SystemExit(f"Input file not found: {input_path}")

    if input_path.is_dir():
        if args.output is not None:
            raise SystemExit("--output can only be used when decoding a single CSV file.")

        raw_csvs = list(iter_csv_inputs(input_path, recursive=args.recursive))
        if not raw_csvs:
            print(f"No raw CSV files found in {input_path}.")
            return 0

        decoded_count = 0
        skipped_count = 0
        for csv_path in raw_csvs:
            output_path = None if args.no_output else default_output_path(csv_path)
            if output_path is not None and output_path.exists() and not args.overwrite:
                skipped_count += 1
                print(f"Skip existing: {output_path}")
                continue

            if decoded_count > 0:
                print()
            decode_csv_file(csv_path, args, output_path)
            decoded_count += 1

        print()
        print(
            f"Batch complete: decoded={decoded_count}, skipped_existing={skipped_count}, "
            f"raw_csvs_seen={len(raw_csvs)}"
        )
        return 0

    suffix = input_path.suffix.lower()
    if suffix == ".kvdat":
        print(summarize_kvdat(input_path))
        return 0
    if suffix != ".csv":
        raise SystemExit("Input must be a .csv logic export or a .kvdat file.")

    output_path = None if args.no_output else (args.output or default_output_path(input_path))
    decode_csv_file(input_path, args, output_path)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
