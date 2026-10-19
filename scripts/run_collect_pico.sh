#!/usr/bin/env bash
# 单臂 Pico 遥操作数据采集启动脚本
#
# 用法:
#   bash scripts/run_collect_pico.sh
#   bash scripts/run_collect_pico.sh runner.num_data_episodes=5
#
# 真机前置条件（缺一不可）:
#   1. Franka Desk 处于可编程模式（FCI 打开）、急停按钮在手边
#   2. 工作区无障碍物；先确认 config 里的
#      reset_ee_pose / ee_pose_limit_min / ee_pose_limit_max 仍匹配当前工作台
#   3. Robotiq 夹爪串口已插入（by-id 路径与 config 一致）
#   4. Pico: XRoboToolkit PC Service 已启动并发布到 ipc:///tmp/vr_data.ipc
#      先用 toolkits/realworld_check/test_pico_data.py 确认有数据
#   5. 不要有正在运行的 Ray 集群: ray stop
#      （Ray 在启动时捕获环境变量，先起的集群拿不到 RLINF_EXT_MODULE）
#   6. 键盘 episode 控制需要能读 /dev/input/event*：
#      sudo setfacl -m u:$USER:rw /dev/input/event*        # 当前会话立即生效
#      sudo usermod -aG input $USER                        # 永久（需重新登录）
#      脚本会自动解析键盘设备；多个候选时可用 RLINF_KEYBOARD_DEVICE 覆盖。

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_PATH="${RELEASE_ROOT}"
EXT_ROOT="${RELEASE_ROOT}/extensions/rig"
CONFIG_ROOT="${RELEASE_ROOT}/release_configs/embodiment"

CONFIG_NAME="${CONFIG_NAME:-realworld_collect_data_pico_single_franka}"

VENV_PATH="${RELEASE_ROOT}/.venv"
if [ ! -x "${VENV_PATH}/bin/python" ]; then
    echo "[错误] 未找到 ${VENV_PATH}，请先在 Release 根目录运行 uv sync" >&2
    exit 1
fi

# 让 collect_data.sh 里的 `python` 解析到 RLinf 自己的 uv 环境
export PATH="${VENV_PATH}/bin:${PATH}"

# rlinf_ext 提供 zed_uvc 相机后端；必须在 worker 启动前就能被 import
export PYTHONPATH="${EXT_ROOT}:${REPO_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
export RLINF_EXT_MODULE="rlinf_ext"
export EMBODIED_CONFIG_PATH="$CONFIG_ROOT"

# 键盘 episode 控制（keyboard_reward_wrapper=start_end_timeout）需要一个明确的
# 键盘设备路径：本机有 2 台"键盘类"设备 —— Dell KB216 真键盘 + BRLTTY 注册的
# 虚拟键盘 —— RLinf 的 KeyboardListener 遇到多个候选会直接抛 RuntimeError。
# 这里自动解析一个（排除 BRLTTY，优先 /dev/input/by-id 稳定路径）；
# 调用方已设 RLINF_KEYBOARD_DEVICE 时尊重调用方。必须在 Ray/worker 起来之前 export。
if [ -z "${RLINF_KEYBOARD_DEVICE:-}" ]; then
    RLINF_KEYBOARD_DEVICE="$("${VENV_PATH}/bin/python" - <<'PY'
import glob
import os

try:
    from evdev import InputDevice, ecodes, list_devices
except Exception:
    raise SystemExit(0)

required = {getattr(ecodes, k) for k in ("KEY_A", "KEY_B", "KEY_C", "KEY_Q")}
candidates = []
for path in sorted(list_devices()):
    try:
        device = InputDevice(path)
    except PermissionError:
        continue
    if required.issubset(set(device.capabilities(verbose=False).get(ecodes.EV_KEY, []))):
        # BRLTTY 注册的是盲文终端用的虚拟键盘，不能当 episode 触发器。
        if "brltty" not in device.name.lower():
            candidates.append((device.name, path))
    device.close()

stable = {os.path.realpath(p): p for p in glob.glob("/dev/input/by-id/*-event-kbd")}
picked = [stable.get(os.path.realpath(path), path) for _, path in candidates]
if len(picked) == 1:
    print(picked[0])
PY
)"
fi

if [ -z "${RLINF_KEYBOARD_DEVICE:-}" ]; then
    echo "[错误] 未能唯一确定键盘设备（可能未授权读取 /dev/input/event*，或有多个候选）。" >&2
    echo "       手动指定后重跑：" >&2
    echo "       RLINF_KEYBOARD_DEVICE=/dev/input/by-id/... bash scripts/run_collect_pico.sh" >&2
    echo "       读取权限：sudo setfacl -m u:\$USER:rw /dev/input/event*" >&2
    exit 1
fi
export RLINF_KEYBOARD_DEVICE

# Optional bounded verification override. Production runs leave it unset and use the
# extension's safe 3000-step (5 min @10Hz) recording ceiling.
if [ -n "${RLINF_KEYBOARD_TIMEOUT_STEPS:-}" ] && ! [[ "${RLINF_KEYBOARD_TIMEOUT_STEPS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[错误] RLINF_KEYBOARD_TIMEOUT_STEPS 必须是正整数，实际为 ${RLINF_KEYBOARD_TIMEOUT_STEPS@Q}" >&2
    exit 1
fi

if command -v ray >/dev/null 2>&1 && ray status >/dev/null 2>&1; then
    echo "[警告] 检测到已在运行的 Ray 集群，它不会带上 RLINF_EXT_MODULE。" >&2
    echo "       请先执行 ray stop 再重跑本脚本。" >&2
fi

echo "[信息] REPO_PATH   = ${REPO_PATH}"
echo "[信息] EXT_MODULE  = ${RLINF_EXT_MODULE}"
echo "[信息] KEYBOARD    = ${RLINF_KEYBOARD_DEVICE}"
echo "[信息] TIMEOUT_STEPS = ${RLINF_KEYBOARD_TIMEOUT_STEPS:-3000}"
echo "[信息] CONFIG_NAME = ${CONFIG_NAME}"

exec bash "${REPO_PATH}/examples/embodiment/collect_data.sh" "${CONFIG_NAME}" "$@"
