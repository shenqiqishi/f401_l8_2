#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
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
            "S{sid} seq={seq} seq_ext={seq_ext} dt={dt}ms ts_ext={ts_ext} wrap(seq={sw},ts={tw})".format(
                sid=sid,
                seq=f.get("seq"),
                seq_ext=f.get("seq_ext"),
                dt=f.get("dt_ms"),
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
    if dt_ms is not None:
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
    if dt_ms is not None:
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
    args = ap.parse_args()

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
    frame_count = 0
    t0 = time.time()
    latest_by_sid = {}
    last_viz_refresh = 0.0

    print(f"listening on {args.port}, {args.baud} 8N1 ... Ctrl+C to stop")

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

                f["seq_ext"] = seq_ext
                f["seq_wrapped"] = seq_wrapped
                f["timestamp_ms_ext"] = tick_ext
                f["timestamp_wrapped"] = tick_wrapped
                f["dt_ms"] = dt_ms

                seq_gap = 0
                if last_seq is not None:
                    seq_delta = (f["seq"] - last_seq) & 0xFFFF
                    # delta==1 means continuous, delta==0 means duplicate/replay.
                    if seq_delta > 1:
                        seq_gap = seq_delta - 1
                f["seq_gap"] = seq_gap

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
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())