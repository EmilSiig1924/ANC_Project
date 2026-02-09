from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import time
import wave
from pathlib import Path

MAGIC = b"ANC1"
HEADER_STRUCT = struct.Struct("<4sHHIIIHHII")
HEADER_SIZE = HEADER_STRUCT.size


def load_pyserial():
    try:
        import serial as pyserial
        import serial.tools.list_ports as list_ports
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: pyserial. Install with: pip install pyserial"
        ) from exc
    return pyserial, list_ports


def choose_port(explicit_port: str | None, list_ports) -> str:
    if explicit_port:
        return explicit_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Connect ESP32-S3 and retry.")
    if len(ports) == 1:
        return ports[0].device

    usb_like = [
        p.device
        for p in ports
        if "USB" in (p.description or "").upper() or "CDC" in (p.description or "").upper()
    ]
    if len(usb_like) == 1:
        return usb_like[0]

    listed = "\n".join(f"- {p.device}: {p.description}" for p in ports)
    raise RuntimeError(
        "Multiple ports detected. Pass --port explicitly.\n"
        f"Available ports:\n{listed}"
    )


def open_serial_port(serial_module, port: str, baud: int):
    """
    Open serial port with control lines deasserted.
    On some ESP32 auto-reset circuits, asserted DTR/RTS can hold/reset the MCU.
    """
    ser = serial_module.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.5
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.setDTR(False)
    ser.setRTS(False)
    return ser


def print_device_line(text: str) -> None:
    print(f"Device: {text}")


def estimate_total_seconds(
    duration_ms: int,
    baud: int,
    sample_rate_hz: int = 48_000,
    channels: int = 1,
) -> float:
    capture_s = duration_ms / 1000.0
    payload_bytes = sample_rate_hz * 2 * channels * capture_s  # 16-bit PCM
    transfer_s = payload_bytes / max(baud / 10.0, 1.0)
    return capture_s + transfer_s


def read_exact(ser, n_bytes: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    chunks = bytearray()
    while len(chunks) < n_bytes:
        if time.monotonic() > deadline:
            raise TimeoutError(f"Timed out while reading {n_bytes} bytes from serial stream.")
        data = ser.read(n_bytes - len(chunks))
        if data:
            chunks.extend(data)
    return bytes(chunks)


def find_and_read_header(ser, timeout_s: float, debug_io: bool = False) -> tuple[int, int, int, int, int]:
    deadline = time.monotonic() + timeout_s
    rolling = bytearray()
    line_buf = bytearray()
    while True:
        if time.monotonic() > deadline:
            raise TimeoutError("Timed out waiting for ANC1 capture header.")
        b = ser.read(1)
        if not b:
            continue
        # Surface textual firmware errors instead of silently waiting for binary forever.
        line_buf.extend(b)
        if b == b"\n":
            text = bytes(line_buf).decode("utf-8", errors="ignore").strip()
            if text and (
                debug_io
                or text.startswith("DBG ")
                or text.startswith("STATUS ")
                or text.startswith("PONG")
                or text.startswith("ANC_CAPTURE_READY")
                or text.startswith("OK ")
            ):
                print_device_line(text)
            if text.startswith("ERR "):
                raise RuntimeError(f"Device reported error: {text}")
            line_buf.clear()

        rolling.extend(b)
        if len(rolling) > len(MAGIC):
            del rolling[:-len(MAGIC)]
        if bytes(rolling) == MAGIC:
            break

    header_tail = read_exact(ser, HEADER_SIZE - len(MAGIC), timeout_s=timeout_s)
    packed = MAGIC + header_tail
    (
        magic,
        version,
        header_bytes,
        sample_rate_hz,
        duration_ms,
        mic_id,
        bits_per_sample,
        channels,
        sample_count,
        payload_bytes,
    ) = HEADER_STRUCT.unpack(packed)

    if magic != MAGIC:
        raise RuntimeError(f"Unexpected header magic: {magic!r}")
    if version != 1:
        raise RuntimeError(f"Unsupported protocol version: {version}")
    if header_bytes != HEADER_SIZE:
        raise RuntimeError(f"Unexpected header size: {header_bytes}, expected {HEADER_SIZE}")
    expected_payload = sample_count * channels * (bits_per_sample // 8)
    if expected_payload != payload_bytes:
        raise RuntimeError(
            f"Payload mismatch. Header says {payload_bytes}, expected {expected_payload}."
        )

    return sample_rate_hz, bits_per_sample, channels, sample_count, payload_bytes


def stream_payload_to_wav(
    ser,
    out_path: Path,
    sample_rate_hz: int,
    bits_per_sample: int,
    channels: int,
    payload_bytes: int,
    timeout_s: float,
) -> None:
    samp_width = bits_per_sample // 8
    out_path.parent.mkdir(parents=True, exist_ok=True)

    remaining = payload_bytes
    chunk_size = 8192
    last_print = 0.0

    payload_deadline = time.monotonic() + timeout_s
    with wave.open(str(out_path), "wb") as wav_out:
        wav_out.setnchannels(channels)
        wav_out.setsampwidth(samp_width)
        wav_out.setframerate(sample_rate_hz)

        while remaining > 0:
            got = ser.read(min(chunk_size, remaining))
            if not got:
                if time.monotonic() > payload_deadline:
                    raise TimeoutError("Timed out while reading PCM payload.")
                continue
            wav_out.writeframesraw(got)
            remaining -= len(got)
            payload_deadline = time.monotonic() + timeout_s

            now = time.monotonic()
            if now - last_print > 0.5:
                done = payload_bytes - remaining
                pct = 100.0 * done / payload_bytes if payload_bytes else 100.0
                print(f"Transfer: {pct:5.1f}% ({done}/{payload_bytes} bytes)")
                last_print = now

        wav_out.writeframes(b"")


def run_preflight_probe(ser, debug_io: bool) -> None:
    ser.reset_input_buffer()
    ser.write(b"STATUS\n")
    ser.flush()
    deadline = time.monotonic() + 1.5
    line_buf = bytearray()
    saw_status = False
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        line_buf.extend(b)
        if b != b"\n":
            continue
        text = bytes(line_buf).decode("utf-8", errors="ignore").strip()
        if text.startswith("STATUS "):
            saw_status = True
            print_device_line(text)
        elif debug_io and text:
            print_device_line(text)
        line_buf.clear()

    if not saw_status:
        print("Warning: no STATUS response seen during preflight probe.")


def build_output_path(out_dir: Path, prefix: str, mic: int) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return out_dir / f"{prefix}_mic{mic}_{timestamp}.wav"


def maybe_run_analysis(
    analyze_script: Path | None,
    analyze_ref: Path | None,
    out_wav: Path,
) -> None:
    if analyze_script is None or analyze_ref is None:
        return
    if not analyze_script.exists():
        raise RuntimeError(f"Analyze script not found: {analyze_script}")
    if not analyze_ref.exists():
        raise RuntimeError(f"Reference WAV not found: {analyze_ref}")

    cmd = [
        sys.executable,
        str(analyze_script),
        "--ref",
        str(analyze_ref),
        "--mic",
        str(out_wav),
    ]
    print("Running analysis:", " ".join(cmd))
    subprocess.run(cmd, check=False)


def main() -> int:
    parser = argparse.ArgumentParser(description="Trigger ESP32-S3 capture and save WAV over USB serial.")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g., COM7). Auto-detected if omitted.")
    parser.add_argument("--baud", type=int, default=115_200, help="Serial baud rate.")
    parser.add_argument("--duration-ms", type=int, default=20_000, help="Recording duration in ms.")
    parser.add_argument(
        "--mic",
        type=int,
        default=1,
        choices=[0, 1, 2],
        help="Mic index: 1/2 for mono capture, 0 for simultaneous stereo capture (ch0=mic1, ch1=mic2).",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("Data"), help="Output directory for captured WAV.")
    parser.add_argument("--prefix", type=str, default="capture", help="Output filename prefix.")
    parser.add_argument("--out", type=Path, default=None, help="Optional explicit output WAV path.")
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=300.0,
        help="Serial read timeout budget in seconds.",
    )
    parser.add_argument("--analyze-script", type=Path, default=None, help="Optional analysis script path.")
    parser.add_argument("--analyze-ref", type=Path, default=None, help="Optional reference WAV for immediate analysis.")
    parser.add_argument("--debug-io", action="store_true", help="Print device text while waiting for binary data.")
    parser.add_argument("--skip-preflight", action="store_true", help="Skip STATUS preflight probe.")
    args = parser.parse_args()

    serial, list_ports = load_pyserial()
    port = choose_port(args.port, list_ports)
    out_path = args.out if args.out is not None else build_output_path(args.out_dir, args.prefix, args.mic)
    command = f"REC {args.duration_ms} MIC={args.mic}\n"

    print(f"Using port: {port}")
    print(f"Sending command: {command.strip()}")
    print(f"Output WAV: {out_path.as_posix()}")
    expected_channels = 2 if args.mic == 0 else 1
    est_total_s = estimate_total_seconds(args.duration_ms, args.baud, channels=expected_channels)
    print(
        f"Expected timing: ~{args.duration_ms / 1000.0:.1f}s capture, "
        f"~{est_total_s:.1f}s total at {args.baud} baud (expected channels={expected_channels})"
    )

    try:
        with open_serial_port(serial, port=port, baud=args.baud) as ser:
            # Give USB CDC moment to settle, then drop boot text.
            time.sleep(0.8)
            if not args.skip_preflight:
                print("Step: preflight STATUS probe...")
                run_preflight_probe(ser, debug_io=args.debug_io)
            else:
                print("Step: preflight skipped (--skip-preflight).")

            ser.reset_input_buffer()
            print(f"Step: triggering device capture (MIC={args.mic}, {args.duration_ms} ms)...")
            ser.write(command.encode("ascii"))
            ser.flush()
            print("Step: waiting for capture header from device...")

            (
                sample_rate_hz,
                bits_per_sample,
                channels,
                sample_count,
                payload_bytes,
            ) = find_and_read_header(ser, timeout_s=args.timeout_s, debug_io=args.debug_io)

            print(
                f"Header: sr={sample_rate_hz} Hz, bits={bits_per_sample}, channels={channels}, "
                f"samples={sample_count}, payload={payload_bytes} bytes"
            )
            print("Step: receiving PCM payload...")

            stream_payload_to_wav(
                ser=ser,
                out_path=out_path,
                sample_rate_hz=sample_rate_hz,
                bits_per_sample=bits_per_sample,
                channels=channels,
                payload_bytes=payload_bytes,
                timeout_s=args.timeout_s,
            )
    except serial.SerialException as exc:
        raise SystemExit(
            f"Failed to open {port}: {exc}\n"
            "Close any serial monitor (VS Code/PlatformIO/Arduino), then retry."
        ) from exc
    except TimeoutError as exc:
        raise SystemExit(
            f"{exc}\n"
            "Use --debug-io to inspect firmware status lines before capture."
        ) from exc
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    print(f"Saved: {out_path.as_posix()}")
    maybe_run_analysis(args.analyze_script, args.analyze_ref, out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
