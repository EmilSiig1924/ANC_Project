from __future__ import annotations

"""
Simple host-side tone player for firmware ANC TONE command.

Use this to validate speaker-path clarity without running full ANC adaptation.
"""

import argparse
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
    saw_status = False
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        print_device_line(line)
        if line.startswith("STATUS "):
            saw_status = True
    if not saw_status:
        print("Warning: no STATUS line seen during preflight probe.")


def stop_anc(ser, timeout_s: float = 1.5) -> None:
    send_line(ser, "ANC STOP")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        print_device_line(line)
        if line.startswith("OK ANC_STOPPED"):
            return
        if line.startswith("ERR "):
            return


def run_tone_once(ser, hz: float, amp: float, ms: int, timeout_s: float) -> None:
    cmd = f"ANC TONE HZ={hz:.3f} AMP={amp:.4f} MS={ms}"
    print(f"Sending: {cmd}")
    send_line(ser, cmd)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        print_device_line(line)
        if line.startswith("ERR "):
            raise RuntimeError(f"Tone command failed: {line}")
        if line.startswith("OK ANC_TONE_DONE"):
            return
    raise TimeoutError("Timed out waiting for OK ANC_TONE_DONE.")


def build_sweep_freqs(start_hz: float, end_hz: float, step_hz: float) -> list[float]:
    if step_hz <= 0.0:
        raise SystemExit("--sweep-step-hz must be > 0.")
    if end_hz < start_hz:
        raise SystemExit("--sweep-end-hz must be >= --sweep-start-hz.")
    out = []
    f = start_hz
    while f <= end_hz + 1e-6:
        out.append(float(f))
        f += step_hz
    return out


# -------------------------
# CLI entry
# -------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Play a clear DAC test tone via firmware ANC TONE command. "
            "This script stops ANC first by default to avoid adaptation noise."
        )
    )
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g., COM7).")
    parser.add_argument("--baud", type=int, default=115_200, help="Serial baud.")
    parser.add_argument("--hz", type=float, default=1000.0, help="Tone frequency for single-tone mode.")
    parser.add_argument("--amp", type=float, default=0.08, help="Tone amplitude (0..0.95).")
    parser.add_argument("--ms", type=int, default=2500, help="Tone duration in ms.")
    parser.add_argument("--repeat", type=int, default=1, help="How many repeats per frequency.")
    parser.add_argument("--gap-ms", type=int, default=250, help="Gap between tones in ms.")
    parser.add_argument("--probe-timeout-s", type=float, default=1.5, help="STATUS probe timeout.")
    parser.add_argument("--timeout-s", type=float, default=20.0, help="Per-tone timeout.")
    parser.add_argument("--stop-anc", action="store_true", help="Stop ANC before tone playback (default).")
    parser.add_argument("--no-stop-anc", dest="stop_anc", action="store_false", help="Do not stop ANC first.")
    parser.set_defaults(stop_anc=True)
    parser.add_argument(
        "--sweep-start-hz",
        type=float,
        default=None,
        help="If set with --sweep-end-hz/--sweep-step-hz, run stepped frequency sweep.",
    )
    parser.add_argument("--sweep-end-hz", type=float, default=None, help="Sweep end frequency.")
    parser.add_argument("--sweep-step-hz", type=float, default=None, help="Sweep step frequency.")
    args = parser.parse_args()

    if args.ms < 100 or args.ms > 60000:
        raise SystemExit("--ms must be in [100, 60000].")
    if args.repeat < 1:
        raise SystemExit("--repeat must be >= 1.")
    if args.gap_ms < 0:
        raise SystemExit("--gap-ms must be >= 0.")
    if not (0.0 < args.amp <= 0.95):
        raise SystemExit("--amp must be in (0, 0.95].")

    use_sweep = (
        args.sweep_start_hz is not None
        or args.sweep_end_hz is not None
        or args.sweep_step_hz is not None
    )
    if use_sweep:
        if args.sweep_start_hz is None or args.sweep_end_hz is None or args.sweep_step_hz is None:
            raise SystemExit("Set all of --sweep-start-hz, --sweep-end-hz, --sweep-step-hz.")
        freqs = build_sweep_freqs(args.sweep_start_hz, args.sweep_end_hz, args.sweep_step_hz)
    else:
        freqs = [args.hz]

    serial, list_ports = load_pyserial()
    port = choose_port(args.port, list_ports)
    print(f"Using port: {port}")
    print(f"Mode: {'sweep' if use_sweep else 'single'}")
    print(f"Amp={args.amp:.4f}, Duration={args.ms} ms, Repeat={args.repeat}, Gap={args.gap_ms} ms")

    try:
        with open_serial_port(serial, port=port, baud=args.baud) as ser:
            ser.reset_input_buffer()
            consume_status_probe(ser, timeout_s=args.probe_timeout_s)
            if args.stop_anc:
                stop_anc(ser)

            total = len(freqs) * args.repeat
            idx = 0
            for hz in freqs:
                for _ in range(args.repeat):
                    idx += 1
                    print(f"\n[{idx}/{total}] Playing {hz:.1f} Hz")
                    run_tone_once(ser, hz=hz, amp=args.amp, ms=args.ms, timeout_s=args.timeout_s)
                    if idx < total and args.gap_ms > 0:
                        time.sleep(args.gap_ms / 1000.0)

    except serial.SerialException as exc:
        raise SystemExit(
            f"Failed to open/use serial port {port}: {exc}\n"
            "Close any serial monitor (VS Code/PlatformIO/Arduino), then retry."
        ) from exc
    except (RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
