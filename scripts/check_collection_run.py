#!/usr/bin/env python3
"""Check whether a collection run actually captured usable teleoperation.

Run this right after a collection attempt — it catches the failure mode where the
keyboard episodes are recorded but nothing teleoperated, which produces episodes
that look valid on disk (``_success.pkl`` + ``demos/*.pt``) yet contain no motion.

The check answers, in order:

1. Did ``demos/`` and ``collected_data/`` produce anything?
2. Did the arm and the gripper actually move inside the recording window?
3. Was ``intervene_action`` non-zero (i.e. was grip held while recording)?
4. Did the rich PICO fields and the second camera land in the pickle?
5. **Did the grip-held window overlap the keyboard recording window?**

Question 5 is the one that matters. ``start_end`` records between ``a`` and ``c``; the
arm only follows the controller while ``grip`` is squeezed past ``control_threshold``.
If the operator teleoperates first and presses ``a``/``c`` afterwards, every episode is
recorded while the controller is idle — empty episodes, no motion, zero actions.

Usage:
    .venv/bin/python scripts/check_collection_run.py            # latest run
    .venv/bin/python scripts/check_collection_run.py --run-dir <path>
"""

from __future__ import annotations

import argparse
import glob
import pickle
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

RELEASE_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = RELEASE_ROOT
LOGS_ROOT = REPO_ROOT / "logs"

#: ``states`` layout: gripper_position(1) + tcp_force(3) + tcp_pose(6) + tcp_torque(3)
#: + tcp_vel(6), concatenated in alphabetical key order.
STATE_SLICES = {
    "gripper_position": slice(0, 1),
    "tcp_force": slice(1, 4),
    "tcp_pose": slice(4, 10),
    "tcp_torque": slice(10, 13),
    "tcp_vel": slice(13, 19),
}

KEYBOARD_RE = re.compile(r"\[INFO (\d\d:\d\d:\d\d) [^\]]*\]\[collect_real_data\.py:\d+\] \[keyboard\] (\w+)")
PICO_RE = re.compile(r"\[INFO (\d\d:\d\d:\d\d) [^\]]*\]\[pico\.py:\d+\] PICO (right controller activated|right controller deactivated)")

OK, BAD, WARN = "  ✅", "  ❌", "  ⚠️"


def seconds(stamp: str) -> int:
    hours, minutes, secs = (int(part) for part in stamp.split(":"))
    return hours * 3600 + minutes * 60 + secs


def clock(value: int) -> str:
    return f"{value // 3600:02d}:{(value % 3600) // 60:02d}:{value % 60:02d}"


def _ordered_events(log: Path, pattern: re.Pattern[str]) -> list[tuple[int, str]]:
    """Return ``(seconds, event)`` pairs in log order.

    Sorting by ``(seconds, event)`` would reshuffle same-second events — ``abort`` and
    ``start`` land on the same second whenever a recording is aborted immediately — so
    the original index is the tie-breaker.
    """
    matches = pattern.findall(log.read_text(errors="ignore"))
    return [(seconds(stamp), event) for _, stamp, event in
            sorted(((index, stamp, event) for index, (stamp, event) in enumerate(matches)))]


def latest_run() -> Path:
    runs = sorted((p for p in LOGS_ROOT.glob("*realworld_collect_data*") if p.is_dir()),
                  key=lambda p: p.stat().st_mtime)
    if not runs:
        raise SystemExit("[错误] RLinf/logs 下没有采集运行目录")
    return runs[-1]


def find_worker_log(run_dir: Path) -> Path | None:
    """Locate the Ray worker log that served this run.

    Worker logs record the replay-buffer path, which contains the run directory, so a
    content match identifies the right file even when several Ray sessions exist.
    """
    try:
        found = subprocess.run(
            ["grep", "-rl", "--include=worker-*.out", str(run_dir), "/tmp/ray"],
            capture_output=True, text=True, timeout=60,
        ).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return None
    return Path(found[0]) if found else None


def load_episodes(run_dir: Path) -> list[tuple[Path, dict]]:
    data_dir = run_dir / "collected_data"
    episodes = []
    for path in sorted(data_dir.glob("*.pkl")):
        with path.open("rb") as handle:
            episodes.append((path, pickle.load(handle)))
    return episodes


def held_windows(log: Path | None) -> list[tuple[int, int]]:
    if log is None or not log.exists():
        return []
    windows, start = [], None
    for stamp, event in _ordered_events(log, PICO_RE):
        # ``deactivated`` ends with ``activated``, so compare the full event name.
        if event == "right controller activated":
            start = stamp
        elif event == "right controller deactivated" and start is not None:
            windows.append((start, stamp))
            start = None
    if start is not None:
        windows.append((start, start + 10**6))
    return windows


def recorded_windows(log: Path | None) -> list[tuple[int, int, str]]:
    if log is None or not log.exists():
        return []
    events = _ordered_events(log, KEYBOARD_RE)
    windows = []
    for index, (stamp, event) in enumerate(events):
        if event != "start":
            continue
        end = next((t for t, e in events[index + 1:] if e in ("end_success", "abort")), None)
        windows.append((stamp, end, event))
    return windows


def overlap(start: int, end: int, windows: list[tuple[int, int]]) -> int:
    return sum(max(0, min(end, hi) - max(start, lo)) for lo, hi in windows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", default=None, help="默认取 RLinf/logs 下最新一次")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve() if args.run_dir else latest_run()
    print(f"  运行目录: {run_dir}")
    if not run_dir.is_dir():
        print(f"{BAD} 目录不存在", file=sys.stderr)
        return 1

    log = find_worker_log(run_dir)
    print(f"  worker 日志: {log if log else '未找到（时间轴检查会跳过）'}")

    problems: list[str] = []

    # ── 1. 两条通道 ────────────────────────────────────────────────────────────
    print("\n── 1. 交付通道 ──")
    demos = sorted((run_dir / "demos").glob("trajectory_*.pt"))
    episodes = load_episodes(run_dir)
    if demos:
        print(f"{OK} demos/ 产出 {len(demos)} 条轨迹")
    else:
        print(f"{BAD} demos/ 没有 trajectory_*.pt")
        problems.append("demos/ 无产出（按 c 成功结束才会写入）")
    if episodes:
        success = sum(1 for p, _ in episodes if p.stem.endswith("_success"))
        print(f"{OK} collected_data/ {len(episodes)} 个 pkl（_success {success} / _fail {len(episodes) - success}）")
    else:
        print(f"{BAD} collected_data/ 没有 pkl")
        problems.append("collected_data/ 无产出")
        print("\n❌ 没有数据可检查")
        return 1

    # ── 2. 每条 episode 的内容 ────────────────────────────────────────────────
    print("\n── 2. episode 内容 ──")
    for path, data in episodes:
        observations = data.get("observations") or []
        infos = data.get("infos") or []
        steps = len(data.get("actions") or [])
        if not observations:
            print(f"{BAD} {path.name}: 没有 observations")
            problems.append(f"{path.name} 无观测")
            continue

        states = np.stack([np.asarray(o["states"], dtype=float) for o in observations])
        spans = {name: float(np.abs(states[:, sl].max(0) - states[:, sl].min(0)).max())
                 for name, sl in STATE_SLICES.items()}

        moved = spans["tcp_pose"] > 1e-3
        gripper_moved = spans["gripper_position"] > 1e-4

        actions = [np.asarray(i.get("intervene_action"), dtype=float)
                   for i in infos if isinstance(i, dict) and i.get("intervene_action") is not None]
        nonzero = sum(1 for a in actions if np.abs(a).sum() > 1e-9)

        print(f"    {path.name}")
        print(f"      录制步数 {steps}  观测 {len(observations)}  infos {len(infos)}")
        print(f"      末端跨度 {spans['tcp_pose']:.5f}  {'✅ 动过' if moved else '❌ 没动'}"
              f"   夹爪跨度 {spans['gripper_position']:.5f}  {'✅ 动过' if gripper_moved else '❌ 没动'}")
        print(f"      intervene_action 非零 {nonzero}/{len(actions)}"
              f"  {'✅' if nonzero else '❌ 全程无遥操作'}")

        if not moved:
            problems.append(f"{path.name}: 录制窗口内机械臂没动")
        if not nonzero:
            problems.append(f"{path.name}: intervene_action 全程为 0（grip 未按）")
        if steps < 20:
            problems.append(f"{path.name}: 只有 {steps} 步（约 {steps / 10:.1f}s），太短，不是有效演示")

    # ── 3. 数据丰富化 ─────────────────────────────────────────────────────────
    print("\n── 3. 数据丰富化 ──")
    # Use the first episode that actually carries observations; a truncated or empty
    # episode would otherwise crash the whole check with an IndexError.
    sample = next((data["observations"][0] for _, data in episodes
                   if data.get("observations")), None)
    if sample is None:
        print(f"{BAD} 所有 episode 都没有 observations")
        problems.append("没有任何 episode 带观测帧")
        return 1
    rich = [k for k in ("pico_position_delta", "pico_rotation_delta")
            if any(isinstance(i, dict) and k in i for _, data in episodes
                   for i in data.get("infos") or [])]
    print(f"{(OK if rich else WARN)} rich PICO 字段: {rich or '缺失'}")
    if not rich:
        problems.append("pickle 里没有 pico_position_delta/rotation_delta")
    has_views = "extra_view_images" in sample
    print(f"{(OK if has_views else WARN)} 第二视角: "
          f"{'extra_view_images ' + str(np.asarray(sample['extra_view_images']).shape) if has_views else '缺失（单相机）'}")

    # ── 4. 时间轴重叠（关键判据） ─────────────────────────────────────────────
    print("\n── 4. 录制窗口 vs grip 按压窗口（关键判据） ──")
    held = held_windows(log)
    recorded = recorded_windows(log)
    if not log:
        # Fail closed: this section is the one that catches the empty-episode case, so
        # an unavailable log is an unknown verdict, never a pass.
        print(f"{BAD} 找不到 worker 日志 —— 关键判据无法执行")
        print("       worker 日志在 /tmp/ray/session_*/logs/ 下，Ray 重启或清理后会丢失。")
        problems.append("关键判据未执行：找不到 worker 日志（/tmp/ray 可能已被清理）")
    elif not held and not recorded:
        print(f"{BAD} 日志里没有 pico/keyboard 事件 —— 关键判据无法执行")
        problems.append("关键判据未执行：日志里没有 pico/keyboard 事件")
    elif not held:
        print(f"{BAD} 只解析到录制窗口、没有 grip 按压事件 —— 关键判据无法执行")
        problems.append("关键判据未执行：日志里没有 PICO 激活/失活事件")
    elif not recorded:
        print(f"{BAD} 只解析到 grip 按压、没有键盘事件 —— 关键判据无法执行")
        problems.append("关键判据未执行：日志里没有 [keyboard] 事件")
    else:
        events: list[tuple[int, str]] = []
        for stamp, event in _ordered_events(log, PICO_RE):
            events.append((stamp, "grip 按下（开始跟随）" if event == "right controller activated"
                           else "grip 松开（停止跟随）"))
        for stamp, event in _ordered_events(log, KEYBOARD_RE):
            events.append((stamp, {"start": "a 开始录制", "end_success": "c 成功结束",
                                   "abort": "a 中止"}.get(event, event)))
        for stamp, label in sorted(events):
            print(f"      {clock(stamp)}  {label}")

        print()
        total_overlap = 0
        for start, end, _ in recorded:
            if end is None:
                print(f"      录制 {clock(start)}~未闭合  跳过")
                continue
            shared = overlap(start, end, held)
            total_overlap += shared
            print(f"      录制 {clock(start)}~{clock(end)}（{end - start}s）"
                  f"  与 grip 按压重叠 {shared}s  {'✅' if shared else '❌ 无重叠 → 记录不到动作'}")
        if recorded and total_overlap == 0:
            problems.append("所有录制窗口都没有与 grip 按压重叠（先遥操作、后按 a/c）")

    # ── 5. 结论 ──────────────────────────────────────────────────────────────
    print("\n── 结论 ──")
    if problems:
        print(f"  ❌ 这次采集不可用，{len(problems)} 个问题：")
        for item in problems:
            print(f"     - {item}")
        print("\n  正确的操作顺序（关键：grip 必须在录制窗口内按住）：")
        print("     1. 按 a 开始录制")
        print("     2. 保持 grip 按住，移动手柄做演示（机械臂此时跟随）")
        print("     3. 按 c 成功结束")
        return 1

    print("  ✅ 采集可用：录制窗口内有真实遥操作，动作非零，机械臂有运动")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
