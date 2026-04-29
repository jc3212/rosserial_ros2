#!/usr/bin/env python3
import argparse
import os
import sys
import time
from datetime import datetime


def hex_dump(data: bytes, width: int = 16) -> str:
    lines = []
    for i in range(0, len(data), width):
        chunk = data[i:i + width]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b <= 126 else "." for b in chunk)
        lines.append(f"{i:08x}: {hex_part:<{width * 3}}  {ascii_part}")
    return "\n".join(lines)


def calc_len_checksum(length_l: int, length_h: int) -> int:
    return (0xFF - ((length_l + length_h) & 0xFF)) & 0xFF


def calc_msg_checksum(topic_id_l: int, topic_id_h: int, payload: bytes) -> int:
    total = (topic_id_l + topic_id_h + sum(payload)) & 0xFF
    return (0xFF - total) & 0xFF


def parse_rosserial_frames(buffer: bytearray):
    """
    从 buffer 中尽可能解析完整 rosserial 帧。
    返回 (frames, consumed_bytes)

    rosserial frame:
      0:  0xFF
      1:  0xFE
      2:  payload_len_l
      3:  payload_len_h
      4:  len_checksum
      5:  topic_id_l
      6:  topic_id_h
      7:  payload...
      -1: msg_checksum

    total_len = 8 + payload_len
    """
    frames = []
    i = 0

    while i <= len(buffer) - 8:
        if not (buffer[i] == 0xFF and buffer[i + 1] == 0xFE):
            i += 1
            continue

        length_l = buffer[i + 2]
        length_h = buffer[i + 3]
        payload_len = length_l | (length_h << 8)
        total_len = 8 + payload_len

        if i + total_len > len(buffer):
            break  # 不完整，等后续数据

        frame = bytes(buffer[i:i + total_len])

        len_checksum = frame[4]
        topic_id_l = frame[5]
        topic_id_h = frame[6]
        topic_id = topic_id_l | (topic_id_h << 8)
        payload = frame[7:7 + payload_len]
        msg_checksum = frame[-1]

        expected_len_checksum = calc_len_checksum(length_l, length_h)
        expected_msg_checksum = calc_msg_checksum(topic_id_l, topic_id_h, payload)

        frames.append({
            "offset": i,
            "frame": frame,
            "payload_len": payload_len,
            "topic_id": topic_id,
            "len_checksum": len_checksum,
            "expected_len_checksum": expected_len_checksum,
            "len_checksum_ok": len_checksum == expected_len_checksum,
            "msg_checksum": msg_checksum,
            "expected_msg_checksum": expected_msg_checksum,
            "msg_checksum_ok": msg_checksum == expected_msg_checksum,
        })

        i += total_len

    return frames, i


def now_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")


def main():
    parser = argparse.ArgumentParser(
        description="监听完整串口原始流，并可选按 rosserial 帧解析。"
    )
    parser.add_argument(
        "--dev", default="/tmp/ttyV1",
        help="监听设备路径，默认 /tmp/ttyV1"
    )
    parser.add_argument(
        "--out", default="capture.bin",
        help="原始流输出文件，默认 capture.bin"
    )
    parser.add_argument(
        "--chunk-size", type=int, default=4096,
        help="单次读取块大小，默认 4096"
    )
    parser.add_argument(
        "--hex", action="store_true",
        help="打印每个 chunk 的十六进制内容"
    )
    parser.add_argument(
        "--frame", action="store_true",
        help="尝试按 rosserial 帧解析并打印结果"
    )
    parser.add_argument(
        "--frame-hex", action="store_true",
        help="打印每帧完整十六进制（可与 --frame 一起使用）"
    )
    parser.add_argument(
        "--max-bytes", type=int, default=0,
        help="抓取多少字节后退出，0 表示持续运行"
    )
    parser.add_argument(
        "--quiet", action="store_true",
        help="减少 chunk 日志输出"
    )

    args = parser.parse_args()

    print(f"[INFO] {now_str()} opening device: {args.dev}")
    print(f"[INFO] {now_str()} raw capture file: {args.out}")

    try:
        fd = os.open(args.dev, os.O_RDONLY | os.O_NOCTTY)
    except OSError as e:
        print(f"[ERROR] failed to open {args.dev}: {e}", file=sys.stderr)
        sys.exit(1)

    total_bytes = 0
    chunk_count = 0
    frame_count = 0
    frame_buf = bytearray()

    try:
        with open(args.out, "wb") as fout:
            while True:
                chunk = os.read(fd, args.chunk_size)
                if not chunk:
                    time.sleep(0.01)
                    continue

                chunk_count += 1
                total_bytes += len(chunk)

                # 原始流落盘
                fout.write(chunk)
                fout.flush()

                if not args.quiet:
                    print(
                        f"[CHUNK {chunk_count}] {now_str()} "
                        f"len={len(chunk)} total={total_bytes}"
                    )

                if args.hex:
                    print(hex_dump(chunk))
                    print()

                if args.frame or args.frame_hex:
                    frame_buf.extend(chunk)
                    frames, consumed = parse_rosserial_frames(frame_buf)

                    for f in frames:
                        frame_count += 1
                        print(
                            f"[FRAME {frame_count}] {now_str()} "
                            f"topic_id={f['topic_id']} "
                            f"payload_len={f['payload_len']} "
                            f"total_len={len(f['frame'])} "
                            f"len_check={'OK' if f['len_checksum_ok'] else 'BAD'} "
                            f"(got=0x{f['len_checksum']:02x}, exp=0x{f['expected_len_checksum']:02x}) "
                            f"msg_check={'OK' if f['msg_checksum_ok'] else 'BAD'} "
                            f"(got=0x{f['msg_checksum']:02x}, exp=0x{f['expected_msg_checksum']:02x})"
                        )
                        if args.frame_hex:
                            print(hex_dump(f["frame"]))
                            print()

                    if consumed > 0:
                        del frame_buf[:consumed]

                if args.max_bytes > 0 and total_bytes >= args.max_bytes:
                    print(f"[INFO] reached max bytes {args.max_bytes}, exit.")
                    break

    except KeyboardInterrupt:
        print("\n[INFO] interrupted by user.")
    finally:
        os.close(fd)
        print(f"[INFO] total chunks={chunk_count}, total bytes={total_bytes}")


if __name__ == "__main__":
    main()