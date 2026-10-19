#!/usr/bin/env python3
"""同步机械臂当前位姿到采集配置，并维护姿态安全窗口的「以复位姿态为中心」不变量。

为什么需要它：

1. ``FrankaEnv.__init__`` 和每次 ``reset()`` 都会把机械臂插值移动到 ``reset_ee_pose``。
   机械臂一旦断电/被手动挪动，这个值就过期了，启动瞬间机械臂会自己开向旧的复位点。
2. ``ee_pose_limit_min/max`` 的**姿态部分** ``[3:]`` 是 ``clip_euler_to_target_window``
   的有效区间，实际含义是「相对起始姿态能转多少」。它必须**以复位姿态为中心**，
   否则可用转角会不对称、甚至极小（曾出现 rz 只剩 10.7°/5.6°，表现为「转手腕末端不动」）。
   本脚本是唯一会修改 ``reset_ee_pose`` 的工具，所以由它一并维护这条不变量：
   ``--apply`` 时按**现有半宽**把姿态窗口平移居中（只平移，尺寸不变，不放宽安全边界）。
3. 位置部分 ``[:3]`` 绑定的是物理工作台，**保持绝对坐标、不随复位点漂移**；
   本脚本只做包含性检查（复位点跑出盒子会导致启动第一步就被裁、末端停在意外位置）。

用法：
    cd Release
    .venv/bin/python scripts/sync_reset_pose.py            # 只看，不写
    .venv/bin/python scripts/sync_reset_pose.py --apply    # 确认后写入
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

RELEASE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = (
    RELEASE_ROOT
    / "release_configs/embodiment/realworld_collect_data_pico_single_franka.yaml"
)
ROBOT_IP = "172.16.0.2"

#: 「居中」的判定阈值：复位姿态相对窗口中心的偏差超过这个弧度就算偏心。
CENTER_TOLERANCE_RAD = 0.1
AXES_ROT = ("rx", "ry", "rz")
AXES_POS = ("x", "y", "z")


def read_current_pose() -> list[float]:
    """Return the TCP pose as ``[x, y, z, rx, ry, rz]`` in the base frame."""
    import numpy as np
    from scipy.spatial.transform import Rotation as R

    import franky

    robot = franky.Robot(ROBOT_IP)
    try:
        state = robot.state
        position = np.asarray(state.O_T_EE.translation, dtype=np.float64)
        euler = R.from_quat(np.asarray(state.O_T_EE.quaternion, dtype=np.float64)).as_euler(
            "xyz"
        )
        return [float(v) for v in np.concatenate([position, euler])]
    finally:
        del robot


def read_vec(text: str, key: str) -> list[float]:
    """Read a 6-element YAML list field as floats."""
    match = re.search(rf"(?m)^[ \t]*{key}:[ \t]*\[([^\]]*)\]", text)
    if not match:
        raise LookupError(key)
    return [float(v) for v in match.group(1).split(",")]


def replace_vec(text: str, key: str, values: list[float]) -> str:
    """Replace a 6-element YAML list field in place.

    6 decimals is far beyond the robot's accuracy but keeps the file readable
    and produces stable diffs.

    只在水平空白上匹配（``[ \\t]`` 而不是 ``\\s``）：``\\s`` 会连同换行一起吃掉，
    把字段后面的空行吞掉，反复写会把文件压缩变形。
    """
    replacement = "[" + ", ".join(f"{v:.6f}" for v in values) + "]"
    text, count = re.subn(
        rf"(?m)^([ \t]*{key}:[ \t]*)\[.*\][ \t]*$", r"\g<1>" + replacement, text
    )
    if count != 1:
        raise LookupError(f"{key}: 期望替换 1 处，实际 {count} 处")
    return text


def headroom(center: float, lower: float, upper: float) -> tuple[float, float] | None:
    """Radians available from ``center`` down to ``lower`` and up to ``upper``.

    Euler angles wrap at ±π, so ``center`` is first shifted by whole turns to
    land inside ``[lower, upper]``. Returns ``None`` when it cannot be placed
    inside, i.e. the pose really is outside the window.
    """
    for shift in (0.0, 2 * math.pi, -2 * math.pi):
        shifted = center + shift
        if lower - 1e-9 <= shifted <= upper + 1e-9:
            return shifted - lower, upper - shifted
    return None


def describe_window(
    reset_pose: list[float], limit_min: list[float], limit_max: list[float]
) -> tuple[bool, list[str]]:
    """Report per-axis headroom; return (centered, lines).

    All three arguments are full 6-element ``[x, y, z, rx, ry, rz]`` vectors.

    「居中」= 窗口中心与复位姿态的偏差在每个轴上都不超过 ``CENTER_TOLERANCE_RAD``。
    这条不变量正是 ``--apply`` 维护的东西：窗口尺寸可以任意，但必须套在复位姿态上。
    """
    centered = True
    lines = ["姿态窗口（应以复位姿态为中心；下面是末端还能往两边转的角度）:"]
    for i, axis in enumerate(AXES_ROT):
        j = i + 3
        reset_euler = reset_pose[j]
        lower, upper = limit_min[j], limit_max[j]
        # 复位姿态相对窗口中心的偏差，按 ±π 环绕后比较
        deviation = (reset_euler - (lower + upper) / 2.0 + math.pi) % (2 * math.pi) - math.pi
        span = headroom(reset_euler, lower, upper)
        if span is None:
            centered = False
            lines.append(
                f"  {axis}: 复位姿态在窗口之外 ⚠️ (窗口 [{lower:.4f}, {upper:.4f}])"
            )
            continue
        down, up = span
        axis_centered = abs(deviation) <= CENTER_TOLERANCE_RAD
        centered &= axis_centered
        flag = "✅" if axis_centered else f"⚠️ 偏心 {math.degrees(deviation):+.1f}°"
        lines.append(
            f"  {axis}: 下 {math.degrees(down):6.1f}°  上 {math.degrees(up):6.1f}°   {flag}"
        )
    return centered, lines


def describe_workspace(
    reset_pose: list[float], limit_min: list[float], limit_max: list[float]
) -> tuple[bool, list[str]]:
    """Report whether the reset point sits inside the absolute position box.

    All three arguments are full 6-element ``[x, y, z, rx, ry, rz]`` vectors.
    """
    inside = True
    lines = ["位置盒余量（绝对坐标，绑定工作台；复位点应在盒内）:"]
    for i, axis in enumerate(AXES_POS):
        down = reset_pose[i] - limit_min[i]
        up = limit_max[i] - reset_pose[i]
        if down < 0 or up < 0:
            inside = False
            lines.append(
                f"  {axis}: 复位点在盒外 ⚠️  (下 {down * 1000:+.0f} mm / 上 {up * 1000:+.0f} mm)"
            )
        else:
            lines.append(f"  {axis}: 下 {down * 1000:5.0f} mm  上 {up * 1000:5.0f} mm  ✅")
    return inside, lines


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="写配置：reset_ee_pose = 当前位姿，并把姿态窗口重新居中（半宽不变）。不加只打印。",
    )
    parser.add_argument(
        "--config", type=Path, default=DEFAULT_CONFIG, help="目标配置文件（默认单臂采集配置）。"
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.02, help="位置差多少米以内算“一致”（默认 0.02）。"
    )
    parser.add_argument(
        "--rot-tolerance",
        type=float,
        default=0.1,
        help="姿态测地角差多少弧度以内算“一致”（默认 0.1）。",
    )
    args = parser.parse_args()

    config = args.config
    if not config.exists():
        print(f"[错误] 找不到配置文件: {config}", file=sys.stderr)
        return 1

    import numpy as np
    from scipy.spatial.transform import Rotation as R

    text = config.read_text()
    try:
        configured = read_vec(text, "reset_ee_pose")
        limit_min = read_vec(text, "ee_pose_limit_min")
        limit_max = read_vec(text, "ee_pose_limit_max")
    except LookupError as exc:
        print(f"[错误] {config} 缺少字段: {exc}", file=sys.stderr)
        return 1

    current = read_current_pose()

    delta_pos = float(np.linalg.norm(np.asarray(current[:3]) - np.asarray(configured[:3])))
    # 欧拉角在 ±π 处会回绕：3.1312 与 -3.0732 相差 2π，却是同一姿态。
    # 直接相减会得到一个巨大的假差值，所以必须比较两个旋转之间的测地角。
    delta_rot = float(
        (R.from_euler("xyz", configured[3:]).inv() * R.from_euler("xyz", current[3:])).magnitude()
    )
    pose_ok = delta_pos <= args.tolerance and delta_rot <= args.rot_tolerance

    print(f"配置里的 reset_ee_pose : {[round(v, 4) for v in configured]}")
    print(f"机械臂当前位姿        : {[round(v, 4) for v in current]}")
    print(f"位置差 {delta_pos * 1000:.1f} mm / 姿态差 {np.degrees(delta_rot):.2f}°")
    print()

    window_ok, window_lines = describe_window(configured, limit_min, limit_max)
    for line in window_lines:
        print(line)
    print()

    inside, box_lines = describe_workspace(configured, limit_min, limit_max)
    for line in box_lines:
        print(line)

    print()
    print(
        f">>> 判定: 位姿={'一致' if pose_ok else '不一致'}  "
        f"姿态窗口={'居中' if window_ok else '偏心'}  "
        f"位置盒={'包含' if inside else '超出'}"
    )

    if not inside:
        print(">>> ⚠️ 复位点不在位置盒内：启动后第一步就会被裁到盒边界，末端会停在意外位置。")
        print(">>>    请先核对 ee_pose_limit_min/max[:3] 是否仍覆盖工作区，再继续。")

    if pose_ok and window_ok and inside:
        print(">>> 无需修改：启动时不会产生复位运动，且姿态窗口以复位姿态为中心。")
        return 0

    if not args.apply:
        print(">>> 加 --apply 可一次做完两件事：")
        print(">>>   ① reset_ee_pose = 当前位姿（消除启动位移）")
        print(">>>   ② 姿态窗口重新居中到新复位姿态（半宽不变，不放宽安全边界）")
        print(f">>>   {sys.executable} {Path(__file__)} --apply")
        return 0

    # ---------------- 写入 ----------------
    # 按现有半宽平移居中：只改「位置」，不改「尺寸」，因此不放松安全边界。
    halfs = [(limit_max[i + 3] - limit_min[i + 3]) / 2.0 for i in range(3)]
    new_min = list(limit_min[:3]) + [current[i + 3] - halfs[i] for i in range(3)]
    new_max = list(limit_max[:3]) + [current[i + 3] + halfs[i] for i in range(3)]

    try:
        text = replace_vec(text, "reset_ee_pose", current)
        text = replace_vec(text, "ee_pose_limit_min", new_min)
        text = replace_vec(text, "ee_pose_limit_max", new_max)
    except LookupError as exc:
        print(f"[错误] 写配置失败: {exc}", file=sys.stderr)
        return 1
    config.write_text(text)

    print(f">>> 已写入 {config.name}")
    print(f"    reset_ee_pose     : {[round(v, 6) for v in current]}")
    print(f"    ee_pose_limit_min : {[round(v, 6) for v in new_min]}")
    print(f"    ee_pose_limit_max : {[round(v, 6) for v in new_max]}")
    print(f"    姿态半宽保持 ±{[round(h, 4) for h in halfs]} rad 不变（只平移，不放宽）")
    print("    位置盒 [:3] 未改动。")
    if not inside:
        print("    ⚠️ 复位点仍在位置盒外，请手工核对位置盒后再启动。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
