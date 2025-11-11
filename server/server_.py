#!/usr/bin/env python3
"""
Simple server to talk to the Arduino client over USB serial.

Features:
 - Auto-detects serial port (tries ports matching common patterns and looks for READY banner)
 - send SET commands: POST /set {"angles": [a1..a7]}
 - get angles:       GET  /angles -> {"angles": [a1..a7]}
 - Interactive CLI fallback for manual testing

Requires: pyserial, Flask (for REST API). If Flask is not installed the script will still offer the interactive CLI.

Place this file at `server/server.py`.
"""
import serial
import argparse
import json
import logging
import sys
import threading
import time
from typing import List, Optional

try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None

try:
    from flask import Flask, jsonify, request
except Exception:
    Flask = None

LOG = logging.getLogger("visarm_server")
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")


class SerialHandler:
    """Handles serial communication with the Arduino client."""

    def __init__(self, port: Optional[str] = None, baud: int = 115200, timeout: float = 1.0):
        if serial is None:
            raise RuntimeError(
                "pyserial is required. Install with: pip install pyserial")
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self._ser = None
        self._lock = threading.Lock()
        self.current_angles: List[float] = [0.0] * 7

    def _open(self, port_name: str):
        LOG.info("Opening serial port %s @ %d", port_name, self.baud)
        self._ser = serial.Serial(port_name, self.baud, timeout=self.timeout)
        # wait a bit for device to boot
        time.sleep(0.5)

    def find_and_connect(self, attempts: int = 5, wait_between: float = 1.0) -> bool:
        """Try to find a likely serial port and connect. Returns True on success."""
        if self.port_name:
            try:
                self._open(self.port_name)
                return True
            except Exception as e:
                LOG.warning("Could not open configured port %s: %s",
                            self.port_name, e)

        for attempt in range(attempts):
            ports = list(list_ports.comports())
            candidates = []
            for p in ports:
                if p.device is None:
                    continue
                # common names used by Arduinos: ttyACM*, ttyUSB*; try also any port with 'Arduino' or 'USB' in description
                if ("Arduino" in (p.description or "")) or p.device.startswith(("/dev/ttyACM", "/dev/ttyUSB")):
                    candidates.append(p.device)

            # add all devices as fallback
            if not candidates:
                candidates = [p.device for p in ports if p.device]

            LOG.info("Attempt %d: trying ports: %s", attempt + 1, candidates)
            for dev in candidates:
                try:
                    self._open(dev)
                    # try to detect READY banner from Arduino
                    t0 = time.time()
                    ready = False
                    while time.time() - t0 < 2.0:
                        line = self._readline()
                        if not line:
                            continue
                        LOG.debug("serial line while probing: %s", line)
                        if "READY" in line:
                            ready = True
                            break
                    if ready:
                        LOG.info("Connected to device %s (READY detected)", dev)
                        self.port_name = dev
                        return True
                    else:
                        LOG.info(
                            "Port %s opened but READY not seen; keeping connection anyway", dev)
                        self.port_name = dev
                        return True
                except Exception as e:
                    LOG.debug("Failed to open %s: %s", dev, e)

            time.sleep(wait_between)

        LOG.error("Could not find or connect to any serial port")
        return False

    def _readline(self) -> Optional[str]:
        if not self._ser:
            return None
        try:
            raw = self._ser.readline()
            if not raw:
                return None
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                line = str(raw)
            return line
        except Exception as e:
            LOG.debug("readline error: %s", e)
            return None

    def _writeline(self, s: str):
        if not self._ser:
            raise RuntimeError("Serial port not open")
        data = (s + "\n").encode("utf-8")
        with self._lock:
            self._ser.write(data)
            self._ser.flush()

    def send_set(self, angles: List[float], wait_ok: float = 1.0) -> dict:
        """Send a SET command. Returns a dict with result and message."""
        if len(angles) != 7:
            return {"ok": False, "error": "angles must have 7 entries"}
        cmd = "SET " + " ".join(f"{float(a):.2f}" for a in angles)
        LOG.info("-> %s", cmd)
        with self._lock:
            try:
                self._writeline(cmd)
            except Exception as e:
                LOG.error("Failed to write to serial: %s", e)
                return {"ok": False, "error": str(e)}

            # read responses for a short window to capture OK or ERR
            t0 = time.time()
            while time.time() - t0 < wait_ok:
                line = self._readline()
                if not line:
                    continue
                LOG.debug("<- %s", line)
                if line.startswith("OK"):
                    return {"ok": True}
                if line.startswith("ERR"):
                    return {"ok": False, "error": line}

        # no explicit OK/ERR; optimistic success
        return {"ok": True, "warning": "no explicit OK received"}

    def get_angles(self, wait_seconds: float = 1.0) -> Optional[List[float]]:
        """Send GET and parse ANGLES reply. Returns list of 7 floats or None."""
        with self._lock:
            try:
                self._writeline("GET")
            except Exception as e:
                LOG.error("Failed to write GET: %s", e)
                return None

            t0 = time.time()
            while time.time() - t0 < wait_seconds:
                line = self._readline()
                if not line:
                    continue
                LOG.debug("<- %s", line)
                if line.startswith("ANGLES"):
                    try:
                        parts = line.split()
                        vals = [float(x) for x in parts[1:]]
                        if len(vals) == 7:
                            self.current_angles = vals
                            return vals
                        else:
                            LOG.warning(
                                "ANGLES line does not have 7 values: %s", line)
                    except Exception as e:
                        LOG.warning("Failed to parse ANGLES: %s (%s)", line, e)
        LOG.warning("No ANGLES response received")
        return None


def create_app(serial_handler: SerialHandler):
    if Flask is None:
        raise RuntimeError(
            "Flask is required for the REST API. Install with: pip install flask")
    app = Flask(__name__)

    @app.route("/angles", methods=["GET"])
    def angles():
        vals = serial_handler.get_angles()
        if vals is None:
            return jsonify({"ok": False, "error": "no response"}), 502
        return jsonify({"ok": True, "angles": vals})

    @app.route("/set", methods=["POST"])
    def set_angles():
        data = request.get_json(force=True, silent=True)
        if not data:
            return jsonify({"ok": False, "error": "expected JSON body"}), 400
        angles = data.get("angles")
        if not isinstance(angles, list) or len(angles) != 7:
            return jsonify({"ok": False, "error": "angles must be list of 7 numbers"}), 400
        result = serial_handler.send_set(angles)
        if result.get("ok"):
            return jsonify({"ok": True})
        else:
            return jsonify({"ok": False, "error": result.get("error")}), 502

    return app


def interactive_loop(serial_handler: SerialHandler):
    print("Interactive mode. Type 'SET a1 ... a7' or 'GET' or 'quit'.")
    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                break
            if line.upper().startswith("SET"):
                parts = line.split()
                try:
                    angles = [float(x) for x in parts[1:]]
                except Exception:
                    print("Invalid angles; use: SET a1 ... a7")
                    continue
                res = serial_handler.send_set(angles)
                print(res)
            elif line.upper().startswith("GET"):
                vals = serial_handler.get_angles()
                print({"angles": vals})
            else:
                print("Unknown command")
    except (KeyboardInterrupt, EOFError):
        print("\nExiting")


def main():
    parser = argparse.ArgumentParser(
        description="visarm server: talk to Arduino client over serial")
    parser.add_argument(
        "--port", help="serial device path (e.g. /dev/ttyACM0). If omitted, auto-detect")
    parser.add_argument("--baud", type=int, default=115200,
                        help="serial baud rate")
    parser.add_argument("--api", action="store_true",
                        help="run REST API (Flask). If omitted, interactive CLI is used")
    parser.add_argument("--host", default="0.0.0.0", help="API bind host")
    parser.add_argument("--listen", type=int, default=5000, help="API port")
    args = parser.parse_args()

    sh = SerialHandler(port=args.port, baud=args.baud)
    ok = sh.find_and_connect()
    if not ok:
        LOG.error("Failed to connect to Arduino. Exiting.")
        sys.exit(1)

    # initial read to populate angles if available
    try:
        sh.get_angles()
    except Exception:
        pass

    if args.api:
        if Flask is None:
            LOG.error("Flask not installed. Install with: pip install flask")
            sys.exit(1)
        app = create_app(sh)
        LOG.info("Starting API on %s:%d", args.host, args.listen)
        # do not use debug mode by default
        app.run(host=args.host, port=args.listen)
    else:
        interactive_loop(sh)


if __name__ == "__main__":
    main()