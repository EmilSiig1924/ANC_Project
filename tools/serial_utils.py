from __future__ import annotations

from typing import Any


def load_pyserial() -> tuple[Any, Any]:
    try:
        import serial as pyserial
        import serial.tools.list_ports as list_ports
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: pyserial. Install with: pip install pyserial"
        ) from exc
    return pyserial, list_ports


def choose_port(explicit_port: str | None, list_ports: Any) -> str:
    if explicit_port:
        return explicit_port

    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Connect ESP32-S3 and retry.")
    if len(ports) == 1:
        return ports[0].device

    usb_like = [
        port.device
        for port in ports
        if "USB" in (port.description or "").upper()
        or "CDC" in (port.description or "").upper()
    ]
    if len(usb_like) == 1:
        return usb_like[0]

    listed = "\n".join(f"- {port.device}: {port.description}" for port in ports)
    raise SystemExit(
        "Multiple serial ports detected. Pass --port explicitly.\n"
        f"Available ports:\n{listed}"
    )


def open_serial_port(serial_module: Any, port: str, baud: int, timeout: float = 0.25) -> Any:
    """Open the serial port with control lines deasserted to avoid ESP32 auto-reset."""
    ser = serial_module.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.setDTR(False)
    ser.setRTS(False)
    return ser


def read_line(ser: Any) -> str | None:
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("utf-8", errors="replace").strip()


def send_line(ser: Any, line: str) -> None:
    ser.write((line.rstrip() + "\n").encode("utf-8"))
    ser.flush()


def print_device_line(text: str) -> None:
    print(f"Device: {text}")
