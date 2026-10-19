#!/usr/bin/env python3
"""采样 Pico 的 ZMQ 数据流，判断数据是否「活着」。

一键启动脚本在启动 RLinf 之前用它把最后一道关。它专门拦这些情况：

* 一条消息都收不到        → 发布器没起来 / socket 不匹配
* 到达速率过低            → 推流不正常
* 位姿与手柄数值**全为 0** → 典型的「发布器启动时没找到设备，之后一直推全 0」

注意：本工具只能证明「数据非 0 且在流动」，**不能**证明你现在戴着头显。
真正判断「能不能驱动机械臂」的最后一关是启动后按住 grip 看机械臂动不动。

用法::

    .venv/bin/python scripts/pico_probe.py --duration 6
    .venv/bin/python scripts/pico_probe.py --json

退出码：``0`` 通过 / ``1`` 不通过 / ``2`` 参数或环境错误。
"""

from __future__ import annotations

import argparse
import json
import sys
import time

DEFAULT_ADDR = "ipc:///tmp/vr_data.ipc"


def sample(addr: str, duration: float) -> dict:
    """Subscribe to ``addr`` and summarise what arrives within ``duration`` seconds."""
    import zmq

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.connect(addr)

    stats = {
        "messages": 0,
        "non_zero": 0,
        "bad_json": 0,
        "max_grip": 0.0,
        "max_trigger": 0.0,
        "buttons_seen": set(),
        "headset_moved": False,
        "first_headset": None,
        "last_headset": None,
    }

    deadline = time.time() + duration
    while time.time() < deadline:
        if socket.poll(300) == 0:
            continue
        try:
            payload = json.loads(socket.recv())
        except Exception:
            stats["bad_json"] += 1
            continue

        stats["messages"] += 1
        headset = [float(v) for v in (payload.get("headset_pose") or [])][:7]
        right = payload.get("right_controller") or {}
        left = payload.get("left_controller") or {}
        positions = [float(v) for v in (right.get("position") or [])]
        positions += [float(v) for v in (left.get("position") or [])]

        if any(abs(v) > 1e-6 for v in headset + positions):
            stats["non_zero"] += 1

        if headset:
            if stats["first_headset"] is None:
                stats["first_headset"] = headset
            stats["last_headset"] = headset

        try:
            stats["max_grip"] = max(stats["max_grip"], float(right.get("grip") or 0.0))
            stats["max_trigger"] = max(stats["max_trigger"], float(right.get("trigger") or 0.0))
        except (TypeError, ValueError):
            pass

        for name, pressed in (payload.get("buttons") or {}).items():
            if pressed:
                stats["buttons_seen"].add(name)

    if stats["first_headset"] and stats["last_headset"]:
        stats["headset_moved"] = any(
            abs(a - b) > 1e-4
            for a, b in zip(stats["first_headset"], stats["last_headset"])
        )

    socket.close(0)
    context.term()
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zmq-addr", default=DEFAULT_ADDR)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--min-rate", type=float, default=30.0, help="最低可接受到达速率 (Hz)")
    parser.add_argument("--json", action="store_true", help="只输出一行 JSON")
    args = parser.parse_args()

    try:
        stats = sample(args.zmq_addr, args.duration)
    except ImportError as exc:
        print(f"[错误] 缺少 pyzmq：{exc}", file=sys.stderr)
        print("       请用 Release 的虚拟环境运行：.venv/bin/python", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"[错误] 无法连接到 {args.zmq_addr}：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    rate = stats["messages"] / args.duration if args.duration > 0 else 0.0
    result = {
        "ok": False,
        "reason": "",
        "messages": stats["messages"],
        "rate_hz": round(rate, 1),
        "non_zero": stats["non_zero"],
        "bad_json": stats["bad_json"],
        "max_grip": round(stats["max_grip"], 3),
        "max_trigger": round(stats["max_trigger"], 3),
        "buttons_seen": sorted(stats["buttons_seen"]),
        "headset_moved": stats["headset_moved"],
    }

    if stats["messages"] == 0:
        result["reason"] = "一条消息都没收到"
    elif rate < args.min_rate:
        result["reason"] = f"到达速率过低 ({rate:.1f} Hz < {args.min_rate} Hz)"
    elif stats["non_zero"] == 0:
        result["reason"] = "所有位姿数值全为 0（发布器可能没找到设备）"
    else:
        result["ok"] = True
        result["reason"] = "数据在流动且数值非 0"

    if args.json:
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result["ok"] else 1

    print(f"  采样时长      : {args.duration:.0f} s")
    print(f"  收到消息      : {stats['messages']} 条（{rate:.1f} Hz）")
    print(f"  数值非 0 的帧 : {stats['non_zero']} / {stats['messages']}")
    print(f"  解析失败      : {stats['bad_json']}")
    print(f"  grip 最大值   : {stats['max_grip']:.3f}")
    print(f"  trigger 最大值: {stats['max_trigger']:.3f}")
    print(f"  按到过的按键  : {sorted(stats['buttons_seen']) or '无'}")
    print(f"  采样期间头显是否移动过: {'是' if stats['headset_moved'] else '否（静止，正常）'}")
    if result["ok"]:
        print(f"\n  ✅ 通过：{result['reason']}")
    else:
        print(f"\n  ❌ 不通过：{result['reason']}")
    # 给启动脚本解析用的一行（人类可读，空格分隔）
    print(
        "PROBE_RESULT "
        f"ok={int(result['ok'])} messages={result['messages']} rate_hz={result['rate_hz']} "
        f"non_zero={result['non_zero']} max_grip={result['max_grip']:.3f} "
        f"max_trigger={result['max_trigger']:.3f} headset_moved={int(result['headset_moved'])}"
    )
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
