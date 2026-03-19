#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
import struct
import sys
import time

import serial


SOF0 = 0xAA
SOF1 = 0x55

FRAME_TYPE_RANGING = 0x01
FRAME_TYPE_ERROR = 0x02

ERROR_CODE_MAP = {
    0x01: "sensor_offline",
    0x02: "uart_tx_fail",
}


def _init_stat():
    return {
        "n": 0,
        "sum": 0.0,
        "sum2": 0.0,
        "min": None,
        "max": None,
    }


def _update_stat(stat, x):
    stat["n"] += 1
    stat["sum"] += x
    stat["sum2"] += x * x
    stat["min"] = x if stat["min"] is None else min(stat["min"], x)
    stat["max"] = x if stat["max"] is None else max(stat["max"], x)


def _print_stability_summary(stability_by_sid, seq_gap_total, frame_count, prefix="[STAB]"):
    print(f"{prefix} frames={frame_count} seq_lost_total={seq_gap_total}")
    if not stability_by_sid:
        print(f"{prefix} no dt_sid samples")
        return

    for sid in sorted(stability_by_sid.keys()):
        st = stability_by_sid[sid]
        n = st["n"]
        if n <= 0:
            continue
        mean = st["sum"] / n
        var = (st["sum2"] / n) - (mean * mean)
        if var < 0:
            var = 0.0
        std = math.sqrt(var)
        print(
            f"{prefix} sid={sid} n={n} dt_sid_mean={mean:.2f}ms "
            f"std={std:.2f}ms min={st['min']}ms max={st['max']}ms"
        )


def analyze_saved_log(log_path):
    frame_count = 0
    seq_gap_total = 0
    stability_by_sid = {}
    last_tick_by_sid = {}

    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                fr = json.loads(line)
            except Exception:
                continue

            frame_count += 1
            seq_gap_total += int(fr.get("seq_gap", 0) or 0)

            sid = fr.get("sensor_id")
            if sid is None:
                continue

            ts = fr.get("timestamp_ms")
            if ts is None:
                continue

            ts = int(ts) & 0xFFFFFFFF
            prev = last_tick_by_sid.get(sid)
            last_tick_by_sid[sid] = ts
            if prev is None:
                continue

            dt_sid = (ts - prev) & 0xFFFFFFFF
            st = stability_by_sid.setdefault(sid, _init_stat())
            _update_stat(st, float(dt_sid))

    _print_stability_summary(stability_by_sid, seq_gap_total, frame_count, prefix="[ANALYZE]")


def _cell_text(zone):
    if zone.get("no_target"):
        return " ----"

    dist = int(zone.get("dist_mm", 0))
    status = int(zone.get("status", 255))

    if status == 0:
        return f"{min(dist, 9999):4d} "

    return f"{min(dist, 999):3d}!*"


def _render_sensor_grid(frame):
    if frame is None:
        lines = ["  (no frame yet)"]
        return lines

    zones = frame.get("zones", [])
    lines = []
    for r in range(8):
        row_cells = []
        for c in range(8):
            idx = r * 8 + c
            if idx < len(zones):
                row_cells.append(_cell_text(zones[idx]))
            else:
                row_cells.append(" ????")
        lines.append(" ".join(row_cells))
    return lines


def render_three_sensor_8x8(latest_by_sid, frame_count, parser):
    parts = ["\x1b[2J\x1b[H"]
    parts.append("TOF 3-Sensor 8x8 Visualization (unit: mm)")
    parts.append("legend: '----' no-target, '*': status!=0")
    parts.append(
        f"frames={frame_count} crc_fail={parser.stats_crc_fail} resync_drop={parser.stats_resync}"
    )
    parts.append("")

    for sid in (0, 1, 2):
        f = latest_by_sid.get(sid)
        if f is None:
            parts.append(f"S{sid}  (no frame yet)")
            parts.append("")
            continue

        parts.append(
            "S{sid} seq={seq} seq_ext={seq_ext} dt={dt}ms dt_sid={dt_sid}ms ts_ext={ts_ext} wrap(seq={sw},ts={tw})".format(
                sid=sid,
                seq=f.get("seq"),
                seq_ext=f.get("seq_ext"),
                dt=f.get("dt_ms"),
                dt_sid=f.get("dt_sid_ms"),
                ts_ext=f.get("timestamp_ms_ext"),
                sw=int(bool(f.get("seq_wrapped"))),
                tw=int(bool(f.get("timestamp_wrapped"))),
            )
        )
        parts.extend(_render_sensor_grid(f))
        parts.append("")

    return "\n".join(parts)


class CounterUnwrapper:
    def __init__(self, bits: int):
        self.mask = (1 << bits) - 1
        self.last_raw = None
        self.last_ext = None

    def update(self, raw: int):
        raw &= self.mask

        if self.last_raw is None:
            self.last_raw = raw
            self.last_ext = raw
            return raw, False, None

        delta = (raw - self.last_raw) & self.mask
        wrapped = raw < self.last_raw
        self.last_ext += delta
        self.last_raw = raw

        return self.last_ext, wrapped, delta


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class ToFFrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.stats_crc_fail = 0
        self.stats_resync = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []

        while True:
            sof_idx = self.buf.find(bytes([SOF0, SOF1]))
            if sof_idx < 0:
                if len(self.buf) > 1:
                    self.stats_resync += len(self.buf) - 1
                    self.buf = self.buf[-1:]
                break

            if sof_idx > 0:
                self.stats_resync += sof_idx
                del self.buf[:sof_idx]

            if len(self.buf) < 16:
                break

            payload_len = self.buf[14] | (self.buf[15] << 8)
            total_len = 16 + payload_len + 2

            if len(self.buf) < total_len:
                break

            frame_bytes = bytes(self.buf[:total_len])
            crc_rx = frame_bytes[-2] | (frame_bytes[-1] << 8)
            crc_calc = crc16_ccitt_false(frame_bytes[2:-2])

            if crc_rx != crc_calc:
                self.stats_crc_fail += 1
                del self.buf[0]
                continue

            frame = self._decode_frame(frame_bytes)
            frames.append(frame)
            del self.buf[:total_len]

        return frames

    @staticmethod
    def _decode_frame(frame: bytes):
        version = frame[2]
        frame_type = frame[3]
        seq = frame[4] | (frame[5] << 8)
        timestamp_ms = struct.unpack_from("<I", frame, 6)[0]
        sensor_id = frame[10]
        resolution_code = frame[11]
        zone_count = frame[12]
        target_index = frame[13]
        payload_len = frame[14] | (frame[15] << 8)
        payload = frame[16:16 + payload_len]

        out = {
            "version": version,
            "frame_type": frame_type,
            "seq": seq,
            "timestamp_ms": timestamp_ms,
            "sensor_id": sensor_id,
            "resolution_code": resolution_code,
            "zone_count": zone_count,
            "target_index": target_index,
            "payload_len": payload_len,
            "raw_len": len(frame),
            "crc_ok": True,
            "host_time": time.time(),
        }

        if frame_type == FRAME_TYPE_RANGING:
            # 每个 zone: dist_u16 + status_u8
            rec_n = payload_len // 3
            zones = []
            for i in range(rec_n):
                off = i * 3
                dist = payload[off] | (payload[off + 1] << 8)
                status = payload[off + 2]
                zones.append({
                    "zone": i,
                    "dist_mm": dist,
                    "status": status,
                    "no_target": (dist == 0xFFFF and status == 0xFF),
                })
            out["kind"] = "ranging"
            out["zones"] = zones

            # 一致性检查，不阻断解析
            out["zone_count_match"] = (zone_count == rec_n)

        elif frame_type == FRAME_TYPE_ERROR:
            out["kind"] = "error"
            if payload_len >= 5:
                err_code = payload[0]
                err_value = struct.unpack_from("<I", payload, 1)[0]
                out["error_code"] = err_code
                out["error_name"] = ERROR_CODE_MAP.get(err_code, "unknown")
                out["error_value"] = err_value
            else:
                out["error_code"] = None
                out["error_name"] = "payload_too_short"
                out["error_value"] = None
        else:
            out["kind"] = "unknown"

        return out


def format_ranging_line(frame, last_seq):
    seq = frame["seq"]
    sid = frame["sensor_id"]
    z = frame["zones"]

    valid = [x["dist_mm"] for x in z if not x["no_target"]]
    no_target_cnt = sum(1 for x in z if x["no_target"])

    if valid:
        dmin = min(valid)
        dmax = max(valid)
        dmean = sum(valid) / len(valid)
        dist_text = f"min={dmin} max={dmax} mean={dmean:.1f}"
    else:
        dist_text = "no valid distance"

    lost = ""
    gap = frame.get("seq_gap", 0)
    if gap:
        lost = f" LOST={gap}"

    timing = ""
    dt_ms = frame.get("dt_ms")
    dt_sid_ms = frame.get("dt_sid_ms")
    if dt_ms is not None and dt_sid_ms is not None:
        timing = f" dt={dt_ms}ms dt_sid={dt_sid_ms}ms"
    elif dt_ms is not None:
        timing = f" dt={dt_ms}ms"

    return (
        f"[R] seq={seq:05d} sid={sid} zone={len(z)} "
        f"no_target={no_target_cnt} {dist_text}{timing}{lost}"
    )


def format_error_line(frame, last_seq):
    seq = frame["seq"]
    sid = frame["sensor_id"]
    code = frame.get("error_code")
    name = frame.get("error_name")
    val = frame.get("error_value")

    lost = ""
    gap = frame.get("seq_gap", 0)
    if gap:
        lost = f" LOST={gap}"

    timing = ""
    dt_ms = frame.get("dt_ms")
    dt_sid_ms = frame.get("dt_sid_ms")
    if dt_ms is not None and dt_sid_ms is not None:
        timing = f" dt={dt_ms}ms dt_sid={dt_sid_ms}ms"
    elif dt_ms is not None:
        timing = f" dt={dt_ms}ms"

    return f"[E] seq={seq:05d} sid={sid} code={code}({name}) value={val}{timing}{lost}"


def main():
    ap = argparse.ArgumentParser(description="STM32 TOF UART frame parser (Linux)")
    ap.add_argument("-p", "--port", required=True, help="serial port, e.g. /dev/ttyACM0 or /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=460800, help="baudrate, default 460800")
    ap.add_argument("--timeout", type=float, default=0.1, help="serial read timeout in seconds")
    ap.add_argument("--jsonl", action="store_true", help="print JSON lines instead of human-readable lines")
    ap.add_argument("--print-zones", action="store_true", help="print each zone detail in human mode")
    ap.add_argument(
        "--viz-3sensor-8x8",
        action="store_true",
        help="real-time terminal visualization for sensor_id 0/1/2 in 8x8 mode",
    )
    ap.add_argument(
        "--save-log",
        default="",
        help="save decoded frames as JSON lines to the given file path",
    )
    ap.add_argument(
        "--analyze-log",
        default="",
        help="offline analyze a saved JSONL log and exit",
    )
    args = ap.parse_args()

    if args.analyze_log:
        analyze_saved_log(args.analyze_log)
        return 0

    if args.viz_3sensor_8x8 and args.jsonl:
        print("--viz-3sensor-8x8 cannot be used with --jsonl", file=sys.stderr)
        return 2

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        )
    except Exception as e:
        print(f"open serial failed: {e}", file=sys.stderr)
        return 1

    parser = ToFFrameParser()
    last_seq = None
    seq_unwrap = CounterUnwrapper(bits=16)
    tick_unwrap = CounterUnwrapper(bits=32)
    tick_unwrap_by_sid = {}
    stability_by_sid = {}
    seq_gap_total = 0
    frame_count = 0
    t0 = time.time()
    latest_by_sid = {}
    last_viz_refresh = 0.0

    print(f"listening on {args.port}, {args.baud} 8N1 ... Ctrl+C to stop")

    log_fp = None
    if args.save_log:
        log_fp = open(args.save_log, "a", encoding="utf-8")
        print(f"saving log to {args.save_log}")

    try:
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue

            frames = parser.feed(chunk)
            for f in frames:
                frame_count += 1

                # Wrap-aware counters for long-running capture sessions.
                seq_ext, seq_wrapped, _ = seq_unwrap.update(f["seq"])
                tick_ext, tick_wrapped, dt_ms = tick_unwrap.update(f["timestamp_ms"])
                sid = f.get("sensor_id")
                if sid not in tick_unwrap_by_sid:
                    tick_unwrap_by_sid[sid] = CounterUnwrapper(bits=32)
                _, _, dt_sid_ms = tick_unwrap_by_sid[sid].update(f["timestamp_ms"])

                f["seq_ext"] = seq_ext
                f["seq_wrapped"] = seq_wrapped
                f["timestamp_ms_ext"] = tick_ext
                f["timestamp_wrapped"] = tick_wrapped
                f["dt_ms"] = dt_ms
                f["dt_sid_ms"] = dt_sid_ms

                seq_gap = 0
                if last_seq is not None:
                    seq_delta = (f["seq"] - last_seq) & 0xFFFF
                    # delta==1 means continuous, delta==0 means duplicate/replay.
                    if seq_delta > 1:
                        seq_gap = seq_delta - 1
                f["seq_gap"] = seq_gap
                seq_gap_total += seq_gap

                if dt_sid_ms is not None:
                    st = stability_by_sid.setdefault(sid, _init_stat())
                    _update_stat(st, float(dt_sid_ms))

                if log_fp is not None:
                    log_fp.write(json.dumps(f, ensure_ascii=False) + "\n")

                if args.viz_3sensor_8x8:
                    if (
                        f.get("kind") == "ranging"
                        and f.get("resolution_code") == 2
                        and f.get("zone_count") == 64
                        and f.get("sensor_id") in (0, 1, 2)
                    ):
                        latest_by_sid[f["sensor_id"]] = f

                    now = time.time()
                    if now - last_viz_refresh >= 0.1:
                        sys.stdout.write(
                            render_three_sensor_8x8(latest_by_sid, frame_count, parser)
                        )
                        sys.stdout.flush()
                        last_viz_refresh = now

                elif args.jsonl:
                    print(json.dumps(f, ensure_ascii=False))
                else:
                    if f["kind"] == "ranging":
                        print(format_ranging_line(f, last_seq))
                        if args.print_zones:
                            for z in f["zones"]:
                                print(
                                    f"  zone={z['zone']:02d} dist_mm={z['dist_mm']:5d} "
                                    f"status={z['status']:3d} no_target={int(z['no_target'])}"
                                )
                    elif f["kind"] == "error":
                        print(format_error_line(f, last_seq))
                    else:
                        print(f"[?] seq={f['seq']} type={f['frame_type']} len={f['raw_len']}")

                last_seq = f["seq"]

            # 每秒打印一次解析器统计
            if time.time() - t0 >= 1.0:
                t0 = time.time()
                if parser.stats_crc_fail or parser.stats_resync:
                    print(
                        f"[STAT] frames={frame_count} crc_fail={parser.stats_crc_fail} "
                        f"resync_drop={parser.stats_resync}"
                    )

    except KeyboardInterrupt:
        pass
    finally:
        if log_fp is not None:
            log_fp.close()
        ser.close()

    _print_stability_summary(stability_by_sid, seq_gap_total, frame_count)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())