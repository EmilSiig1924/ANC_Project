from __future__ import annotations

"""
Fast ANC delay sweep utility.

Default behavior is intentionally short:
- tests only a few delay values
- uses short calibration/adaptation windows
- ranks by FROZEN error-mic level with optional saturation penalty
"""

import argparse
import re
import statistics
import time

from serial_utils import (
    choose_port,
    load_pyserial,
    open_serial_port,
    read_line,
    send_line,
)


# Parse just the fields needed to score each delay trial.
LOG_RE = re.compile(
    r"LOG ANC(?: MODE=[A-Z]+)? PHASE=([A-Z]+)\s+REF_RMS_DBFS=([-0-9.]+)\s+ERR_RMS_DBFS=([-0-9.]+).*SAT=([0-9]+)"
)


def stop_anc(ser) -> None:
    send_line(ser, "ANC STOP")
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        if line.startswith("OK ANC_STOPPED") or line.startswith("ERR "):
            return


# -------------------------
# ANC trial helpers
# -------------------------

def build_start_cmd(delay: int, args) -> str:
    sign = 1.0 if args.sign >= 0.0 else -1.0
    if args.direct:
        return (
            "ANC DIRECT START "
            f"GAIN={args.gain:.4f} LIMIT_DBFS={args.limit_dbfs:.2f} "
            f"SIGN={sign:.1f} LOG_MS={args.log_ms} DELAY={delay} "
            f"HP_HZ={args.direct_hp_hz:.1f} LP_HZ={args.direct_lp_hz:.1f} "
            f"GATE_DBFS={args.direct_gate_dbfs:.1f}"
        )

    cmd = (
        "ANC START "
        f"CAL_MS={args.cal_ms} ADAPT_MS={args.adapt_ms} "
        f"MU={args.mu:.6f} MUCAL=0.120000 GAIN={args.gain:.4f} "
        f"LIMIT_DBFS={args.limit_dbfs:.2f} SIGN={sign:.1f} LOG_MS={args.log_ms} "
        f"DELAY={delay} LEAK={args.leak:.6f} ADAPT_FLOOR_DBFS={args.adapt_floor_dbfs:.1f}"
    )
    if args.cal_amp is not None:
        cmd += f" CAL_AMP={args.cal_amp:.4f}"
    return cmd


def wait_for_frozen(ser, timeout_s: float, quiet: bool) -> str | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        if not quiet:
            print(f"Device: {line}")
        if line.startswith("ERR "):
            return line
        if line.startswith("OK ANC_PHASE=FROZEN"):
            return None
    return "Timeout waiting for FROZEN"


def measure_frozen_logs(ser, measure_ms: int, quiet: bool) -> list[tuple[float, float, int]]:
    deadline = time.monotonic() + (measure_ms / 1000.0)
    rows: list[tuple[float, float, int]] = []
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        if not quiet:
            print(f"Device: {line}")
        m = LOG_RE.search(line)
        if m and m.group(1) == "FROZEN":
            rows.append((float(m.group(2)), float(m.group(3)), int(m.group(4))))
    return rows


def run_trial(ser, delay: int, args):
    stop_anc(ser)
    ser.reset_input_buffer()
    cmd = build_start_cmd(delay, args)
    print(f"\nDELAY={delay} -> {cmd}")
    send_line(ser, cmd)

    timeout_s = 3.0 if args.direct else max((args.cal_ms + args.adapt_ms) / 1000.0 + 4.0, 6.0)
    err = wait_for_frozen(ser, timeout_s=timeout_s, quiet=args.quiet)
    if err:
        stop_anc(ser)
        return {"delay": delay, "ok": False, "reason": err}

    rows = measure_frozen_logs(ser, measure_ms=args.measure_ms, quiet=args.quiet)
    stop_anc(ser)
    if len(rows) < 2:
        return {"delay": delay, "ok": False, "reason": "Not enough FROZEN logs"}

    ref = [r[0] for r in rows]
    err_vals = [r[1] for r in rows]
    sat_delta = rows[-1][2] - rows[0][2]
    ref_mean = statistics.fmean(ref)
    err_mean = statistics.fmean(err_vals)
    score = err_mean + args.sat_penalty * sat_delta
    print(f"Result: REF={ref_mean:.2f} ERR={err_mean:.2f} SAT_DELTA={sat_delta} SCORE={score:.2f}")
    return {
        "delay": delay,
        "ok": True,
        "ref_mean": ref_mean,
        "err_mean": err_mean,
        "sat_delta": sat_delta,
        "score": score,
        "n": len(rows),
    }


# -------------------------
# CLI entry
# -------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Short ANC delay sweep helper.")
    parser.add_argument("--port", type=str, default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay-start", type=int, default=0)
    parser.add_argument("--delay-end", type=int, default=16)
    parser.add_argument("--delay-step", type=int, default=8)
    parser.add_argument("--cal-ms", type=int, default=1200)
    parser.add_argument("--adapt-ms", type=int, default=2000)
    parser.add_argument("--measure-ms", type=int, default=1200)
    parser.add_argument("--mu", type=float, default=0.002)
    parser.add_argument("--gain", type=float, default=0.2)
    parser.add_argument("--limit-dbfs", type=float, default=-12.0)
    parser.add_argument("--sign", type=float, default=-1.0)
    parser.add_argument("--leak", type=float, default=0.0001)
    parser.add_argument("--adapt-floor-dbfs", type=float, default=-68.0)
    parser.add_argument("--log-ms", type=int, default=400)
    parser.add_argument("--cal-amp", type=float, default=None)
    parser.add_argument("--sat-penalty", type=float, default=0.05)
    parser.add_argument("--direct-hp-hz", type=float, default=70.0)
    parser.add_argument("--direct-lp-hz", type=float, default=2200.0)
    parser.add_argument("--direct-gate-dbfs", type=float, default=-52.0)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--start-best", action="store_true")
    parser.add_argument("--direct", action="store_true", help="Sweep/start using ANC DIRECT START mode.")
    args = parser.parse_args()

    if args.delay_step <= 0:
        raise SystemExit("--delay-step must be > 0")
    if args.delay_end < args.delay_start:
        raise SystemExit("--delay-end must be >= --delay-start")
    delays = list(range(args.delay_start, args.delay_end + 1, args.delay_step))
    if not delays:
        raise SystemExit("No delay values to test.")

    serial, list_ports = load_pyserial()
    port = choose_port(args.port, list_ports)
    print(f"Using port: {port}")
    print(f"Mode: {'DIRECT' if args.direct else 'FXLMS'}")
    print(f"Sweeping delays: {delays}")

    results = []
    try:
        with open_serial_port(serial, port=port, baud=args.baud) as ser:
            send_line(ser, "STATUS")
            time.sleep(0.3)
            for d in delays:
                results.append(run_trial(ser, d, args))
    except serial.SerialException as exc:
        raise SystemExit(f"Serial error on {port}: {exc}") from exc

    ok = [r for r in results if r.get("ok")]
    bad = [r for r in results if not r.get("ok")]
    if not ok:
        print("\nNo passing trials.")
        for r in bad:
            print(f"- DELAY={r['delay']}: {r['reason']}")
        return 1

    ranked = sorted(ok, key=lambda r: r["score"])
    print("\n=== Summary (best first) ===")
    for r in ranked:
        print(
            f"- DELAY={r['delay']:>3} ERR={r['err_mean']:.2f} REF={r['ref_mean']:.2f} "
            f"SAT_DELTA={r['sat_delta']:>4} N={r['n']:>2} SCORE={r['score']:.2f}"
        )
    for r in bad:
        print(f"- DELAY={r['delay']:>3} FAIL: {r['reason']}")

    best = ranked[0]
    print(f"\nBest delay = {best['delay']}")
    if args.start_best:
        cmd = build_start_cmd(best["delay"], args)
        print(f"Starting best config: {cmd}")
        try:
            with open_serial_port(serial, port=port, baud=args.baud) as ser:
                send_line(ser, cmd)
                time.sleep(0.6)
        except serial.SerialException:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
