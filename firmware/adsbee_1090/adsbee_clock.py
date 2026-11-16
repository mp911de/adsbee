#!/usr/bin/env python3
"""Measure ADSBee USB command latency and GNSS clock offset without changing settings."""

import argparse
import csv
import fcntl
import json
import math
import os
import select
import statistics
import termios
import time
import tty


def clock_pair():
    before = time.monotonic_ns()
    wall = time.time_ns()
    after = time.monotonic_ns()
    return wall, (before + after) // 2


def measurement(start, end, epoch_ms=None):
    wall_start, mono_start = start
    wall_end, mono_end = end
    rtt = (mono_end - mono_start) / 1e6
    if abs((wall_end - wall_start) - (mono_end - mono_start)) > 1_000_000:
        raise ValueError("Host clock changed during query")
    result = {"host_send_ns": wall_start, "host_receive_ns": wall_end, "rtt_ms": rtt}
    if epoch_ms is not None:
        offset = (epoch_ms * 1_000_000 - (wall_start + wall_end) // 2) / 1e6
        result.update(epoch_ms=epoch_ms, offset_ms=offset,
                      transport_bound_ms=rtt / 2 + 1)
    return result


def parse_fix(line):
    fields = line.removeprefix("GNSS_FIX=").split(",")
    if len(fields) != 10 or not fields[9].isdigit():
        raise ValueError("GNSS_FIX lacks valid epoch milliseconds")
    return int(fields[9]), int(fields[8]), int(fields[0])


class Port:
    def __init__(self, path):
        self.path = path

    def __enter__(self):
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.saved = None
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.saved = termios.tcgetattr(self.fd)
            tty.setraw(self.fd, termios.TCSANOW)
            attrs = termios.tcgetattr(self.fd)
            attrs[2] |= termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIFLUSH)
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *args):
        try:
            if self.saved is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
        finally:
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            finally:
                os.close(self.fd)

    def query(self, command, prefix, timeout=2):
        # One request at a time. A timeout aborts the run to avoid matching a late reply.
        termios.tcflush(self.fd, termios.TCIFLUSH)
        start = clock_pair()
        payload = (command + "\r\n").encode("ascii")
        if os.write(self.fd, payload) != len(payload):
            raise IOError("Partial command write")
        deadline = time.monotonic() + timeout
        pending = b""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.fd], [], [], remaining)[0]:
                raise TimeoutError(command + " timed out")
            data = os.read(self.fd, 4096)
            end = clock_pair()
            if not data:
                raise IOError("Device disconnected")
            pending += data
            while b"\n" in pending:
                raw, pending = pending.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").strip()
                if line.startswith(prefix):
                    return line, start, end
                if line.startswith("ERROR"):
                    raise RuntimeError(line)
            if len(pending) > 65536:
                raise IOError("Unterminated device output")


def distribution(values):
    ordered = sorted(values)
    return {"min": ordered[0], "median": statistics.median(ordered),
            "p95": ordered[math.ceil(len(ordered) * .95) - 1], "max": ordered[-1]}


def self_test():
    base = 1_800_000_000_000_000_000
    start = (base, 0)
    end = (base + 20_000_000, 20_000_000)
    assert measurement(start, end, base // 1_000_000 + 2010)["offset_ms"] == 2000
    asymmetric = measurement(start, end, base // 1_000_000 + 2004)
    assert asymmetric["offset_ms"] == 1994
    assert abs(asymmetric["offset_ms"] - 2000) <= asymmetric["transport_bound_ms"]
    assert parse_fix("GNSS_FIX=1,0,0,0,0,0,8,00:00:00,42,1800000000000") == (1800000000000, 42, 1)
    for line in ("GNSS_FIX=1,0,0,0,0,0,8,00:00:00,42,", "GNSS_FIX=1,0,0"):
        try:
            parse_fix(line)
        except ValueError:
            pass
        else:
            raise AssertionError("Missing time accepted")
    try:
        measurement(start, (base + 1_020_000_000, 20_000_000))
    except ValueError:
        pass
    else:
        raise AssertionError("Clock step accepted")
    print("PASS: offset sign, asymmetric delay, valid epoch, missing epoch, clock step")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/cu.usbmodem2101")
    parser.add_argument("--baseline", type=int, default=15)
    parser.add_argument("--samples", type=int, default=15)
    parser.add_argument("--interval", type=float, default=.1)
    parser.add_argument("--csv", default="/private/tmp/adsbee-clock.csv")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.baseline < 1 or args.samples < 1 or args.interval < 0:
        parser.error("Counts must be positive and interval nonnegative")
    fields = ["phase", "sample", "host_send_ns", "host_receive_ns", "rtt_ms",
              "epoch_ms", "offset_ms", "transport_bound_ms", "pps", "fix_valid"]
    baseline, fixes, rejected = [], [], 0
    with Port(args.port) as port, open(args.csv, "w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for phase, count in (("uptime", args.baseline), ("gnss", args.samples)):
            print(f"Sampling {phase}: {count} queries", flush=True)
            for i in range(count):
                command = "AT+UPTIME?" if phase == "uptime" else "AT+GNSS_FIX?"
                prefix = "UPTIME=" if phase == "uptime" else "GNSS_FIX="
                line, start, end = port.query(command, prefix)
                try:
                    if phase == "gnss":
                        epoch, pps, valid = parse_fix(line)
                        row = measurement(start, end, epoch)
                        row.update(pps=pps, fix_valid=valid)
                        fixes.append(row)
                    else:
                        row = measurement(start, end)
                        baseline.append(row)
                except ValueError as error:
                    rejected += 1
                    print(f"Rejected {phase} sample {i}: {error}", flush=True)
                else:
                    writer.writerow(dict(row, phase=phase, sample=i))
                    output.flush()
                time.sleep(args.interval)
    summary = {"baseline_samples": len(baseline), "gnss_samples": len(fixes), "rejected": rejected}
    if baseline:
        summary["uptime_rtt_ms"] = distribution([r["rtt_ms"] for r in baseline])
    if fixes:
        best = sorted(fixes, key=lambda r: r["rtt_ms"])[:max(1, len(fixes) // 10)]
        summary.update(
            gnss_rtt_ms=distribution([r["rtt_ms"] for r in fixes]),
            offset_ms=distribution([r["offset_ms"] for r in fixes]),
            fastest_decile_offset_ms=statistics.median([r["offset_ms"] for r in best]),
            fastest_sample_transport_bound_ms=best[0]["transport_bound_ms"],
            fastest_sample_offset_ms=best[0]["offset_ms"],
            pps_first=fixes[0]["pps"], pps_last=fixes[-1]["pps"],
            fix_valid_samples=sum(r["fix_valid"] == 1 for r in fixes),
        )
    print(json.dumps(summary, indent=2))
    print("CSV:", args.csv)
    if not fixes:
        raise SystemExit("No valid GNSS time measurements")


if __name__ == "__main__":
    main()
