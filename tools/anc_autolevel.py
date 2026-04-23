from __future__ import annotations

"""
Host helper for ANC AUTOLEVEL.

Runs AUTOLEVEL on firmware, parses tuned amplitudes, and can optionally
start ANC immediately with the tuned defaults.
"""

import argparse
import re
import time

from serial_utils import (
    choose_port,
    load_pyserial,
    open_serial_port,
    print_device_line,
    read_line,
    send_line,
)


def consume_status_probe(ser, timeout_s: float) -> None:
    send_line(ser, "STATUS")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        print_device_line(line)
        if line.startswith("STATUS "):
            return


def run_autolevel(ser, cmd: str, timeout_s: float) -> tuple[float | None, float | None, str]:
    send_line(ser, cmd)
    deadline = time.monotonic() + timeout_s
    result = "FAIL"
    tone_amp = None
    cal_amp = None
    pattern = re.compile(r"TONE_AMP=([0-9]*\.?[0-9]+)\s+CAL_AMP=([0-9]*\.?[0-9]+)")

    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        print_device_line(line)
        if line.startswith("ERR "):
            raise RuntimeError(f"Device reported error: {line}")
        if line.startswith("OK ANC_AUTOLEVEL_DONE"):
            m = pattern.search(line)
            if m:
                tone_amp = float(m.group(1))
                cal_amp = float(m.group(2))
        if line.startswith("AUTOLEVEL RESULT="):
            result = line.split("=", 1)[1].strip().upper()
        if line == "AUTOLEVEL END":
            return tone_amp, cal_amp, result

    raise TimeoutError("Timed out waiting for AUTOLEVEL END.")


def run_start(ser, log_ms: int, timeout_s: float) -> None:
    send_line(ser, f"ANC START LOG_MS={log_ms}")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        print_device_line(line)
        if line.startswith("ERR "):
            raise RuntimeError(f"ANC START failed: {line}")
        if line.startswith("OK ANC_STARTED"):
            return
    raise TimeoutError("Timed out waiting for ANC START response.")


# -------------------------
# CLI entry
# -------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run ANC AUTOLEVEL to tune a quiet DAC tone amplitude from mic feedback. "
            "The tuned CAL_AMP is then used by subsequent ANC START defaults."
        )
    )
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g., COM7).")
    parser.add_argument("--baud", type=int, default=115_200, help="Serial baud.")
    parser.add_argument("--target-db", type=float, default=10.0, help="Target mic rise above baseline in dB.")
    parser.add_argument("--hz", type=float, default=1000.0, help="Tone frequency in Hz.")
    parser.add_argument("--base-ms", type=int, default=1000, help="Baseline window in ms.")
    parser.add_argument("--tone-ms", type=int, default=1000, help="Tone measurement window in ms.")
    parser.add_argument("--start-amp", type=float, default=0.05, help="Initial tone amplitude (0..1).")
    parser.add_argument("--min-amp", type=float, default=0.005, help="Lower amplitude clamp.")
    parser.add_argument("--max-amp", type=float, default=0.35, help="Upper amplitude clamp.")
    parser.add_argument("--iter", type=int, default=8, help="Max tuning iterations.")
    parser.add_argument("--timeout-s", type=float, default=45.0, help="AUTOLEVEL timeout budget.")
    parser.add_argument("--probe-timeout-s", type=float, default=1.5, help="Preflight STATUS probe timeout.")
    parser.add_argument("--log-ms", type=int, default=1000, help="If --start-anc, use this ANC log period.")
    start_group = parser.add_mutually_exclusive_group()
    start_group.add_argument(
        "--start-anc",
        dest="start_anc",
        action="store_true",
        help="Start ANC immediately after successful/warn auto-level.",
    )
    start_group.add_argument(
        "--no-start-anc",
        dest="start_anc",
        action="store_false",
        help="Only tune levels; do not start ANC.",
    )
    parser.set_defaults(start_anc=True)
    args = parser.parse_args()

    serial, list_ports = load_pyserial()
    port = choose_port(args.port, list_ports)
    print(f"Using port: {port}")

    cmd = (
        "ANC AUTOLEVEL "
        f"TARGET_DB={args.target_db:.3f} "
        f"HZ={args.hz:.3f} "
        f"BASE_MS={args.base_ms} "
        f"TONE_MS={args.tone_ms} "
        f"START_AMP={args.start_amp:.5f} "
        f"MIN_AMP={args.min_amp:.5f} "
        f"MAX_AMP={args.max_amp:.5f} "
        f"ITER={args.iter}"
    )
    print(f"Sending: {cmd}")

    try:
        with open_serial_port(serial, port=port, baud=args.baud) as ser:
            ser.reset_input_buffer()
            consume_status_probe(ser, timeout_s=args.probe_timeout_s)

            tone_amp, cal_amp, result = run_autolevel(ser, cmd=cmd, timeout_s=args.timeout_s)
            if tone_amp is not None and cal_amp is not None:
                print(
                    f"Tuned values: TONE_AMP={tone_amp:.4f}, CAL_AMP={cal_amp:.4f}, RESULT={result}"
                )
            else:
                print(f"Auto-level completed but no parsed amp line found. RESULT={result}")

            if result not in {"PASS", "WARN"}:
                raise RuntimeError(f"AUTOLEVEL reported RESULT={result}")

            if args.start_anc:
                run_start(ser, log_ms=args.log_ms, timeout_s=5.0)
                print("ANC started with tuned defaults.")
            else:
                print("ANC not started (--no-start-anc).")

            send_line(ser, "STATUS")
            status_deadline = time.monotonic() + 2.0
            while time.monotonic() < status_deadline:
                line = read_line(ser)
                if not line:
                    continue
                print_device_line(line)
                if line.startswith("STATUS ANC_RUNTIME_CAL_AMP_TUNED="):
                    break

    except serial.SerialException as exc:
        raise SystemExit(
            f"Failed to open/use serial port {port}: {exc}\n"
            "Close any serial monitor (VS Code/PlatformIO/Arduino), then retry."
        ) from exc
    except (RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
