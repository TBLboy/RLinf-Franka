#!/usr/bin/env bash
# =============================================================================
# run_pico_session.sh —— Pico VR 遥操作 Franka 采集：一键启动 + Ctrl-C 一键收尾
#
# 用法：
#   bash scripts/run_pico_session.sh              # 正常启动（推荐）
#   bash scripts/run_pico_session.sh --check      # 只预检+位姿核对，不启动服务、不碰机械臂运动
#   bash scripts/run_pico_session.sh --dry-run    # 完整走一遍但只打印，不真启动任何东西
#   bash scripts/run_pico_session.sh [hydra 覆盖参数...]
#        例：bash scripts/run_pico_session.sh env.eval.max_episode_steps=3000
#
# 流程（7 个阶段，编号从 0 开始）：
#   0 预检        —— 关键路径、网络、清理残留 Ray
#   1 前置服务    —— 启动 XRoboToolkit PC Service（已在跑则跳过）
#   2 机械臂检查  —— 只读读位姿，核对 reset_ee_pose（不产生运动）
#   3 【暂停】    —— 你戴头显、站好位、完成头显内设置，然后按回车
#   4 发布器      —— 启动 vr_data_publisher，等日志出现 device found
#   5 数据探针    —— 采样 ZMQ，确认数据非 0 且在流动
#   6 启动采集    —— 前台运行 RLinf（Ctrl-C 即停）
#
# 收尾：任何退出路径（正常结束 / Ctrl-C / 出错）都会自动执行
#   停发布器 → ray stop --force → 停 PC Service
#
# ⚠️ 安全：第 6 步启动采集时，机械臂会插值移动到 reset_ee_pose。
#          请先清空工作区，并把急停按钮放在手边。
# ⚠️ 采集运行期间，不要从其它进程连接机械臂（见文档第 7 章 7.25）。
# =============================================================================
set -uo pipefail

# -----------------------------------------------------------------------------
# 参数
# -----------------------------------------------------------------------------
DRY_RUN=0
CHECK_ONLY=0
HYDRA_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY_RUN=1 ;;
    --check)   CHECK_ONLY=1 ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *)         HYDRA_ARGS+=("$1") ;;
  esac
  shift
done

# -----------------------------------------------------------------------------
# 路径
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RLINF_DIR="$WORK_ROOT"
VENV="$WORK_ROOT/.venv"
PY="$VENV/bin/python"
RAY="$VENV/bin/ray"
WRAPPER="$SCRIPT_DIR/run_collect_pico.sh"
CONFIG_ROOT="$WORK_ROOT/release_configs/embodiment"
SYNC_TOOL="$SCRIPT_DIR/sync_reset_pose.py"
PROBE="$SCRIPT_DIR/pico_probe.py"

PICO_DIR="$WORK_ROOT/pico_bridge"
# XRoboToolkit's vendor binding is installed in the proven PC-Service sample
# environment. It is external to Release until its licensed installer can be
# reproduced here; callers may override the path for a compatible replacement.
PICO_VENV="${RLINF_PICO_SDK_VENV:-$WORK_ROOT/../pico_software/XRoboToolkit-Teleop-Sample-Python/.venv}"
PICO_CFG_NAME="configs/vr_bridge.yaml"

PC_SERVICE_DIR="/opt/apps/roboticsservice"
ROBOT_IP="172.16.0.2"
ZMQ_ADDR="ipc:///tmp/vr_data.ipc"

LOG_DIR="/tmp/pico_session"
SESSION_LOG="$LOG_DIR/session_$(date +%Y%m%d-%H%M%S).log"
PUB_LOG="$LOG_DIR/publisher.log"
PCS_LOG="$LOG_DIR/pcservice.log"
mkdir -p "$LOG_DIR"

PC_IP=""
PUB_PID=""
CLEANED=0

# -----------------------------------------------------------------------------
# 输出
# -----------------------------------------------------------------------------
C_R=$'\033[31m'; C_G=$'\033[32m'; C_Y=$'\033[33m'; C_C=$'\033[36m'; C_0=$'\033[0m'

say()  { printf '%s\n' "$*" | tee -a "$SESSION_LOG"; }
ok()   { printf '%s%s%s\n' "$C_G" "$*" "$C_0" | tee -a "$SESSION_LOG"; }
warn() { printf '%s%s%s\n' "$C_Y" "$*" "$C_0" | tee -a "$SESSION_LOG"; }
err()  { printf '%s%s%s\n' "$C_R" "$*" "$C_0" | tee -a "$SESSION_LOG"; }
hr()   { say "────────────────────────────────────────────────────────────────"; }
die()  { err ""; err "❌ $*"; exit 1; }

pause_enter() {
  local title="${1:-就绪后}"
  if [ "$DRY_RUN" = 1 ]; then say "  [dry-run] 跳过等待：$title"; return 0; fi
  printf '\n%s%s%s\n' "$C_C" "$title" "$C_0" | tee -a "$SESSION_LOG"
  printf '按【回车】继续（Ctrl-C 取消并收尾）... '
  read -r _ || true
  say ""
}

# -----------------------------------------------------------------------------
# 收尾（所有退出路径都会走这里）
# -----------------------------------------------------------------------------
cleanup() {
  [ "$CLEANED" = 1 ] && return 0
  CLEANED=1
  hr
  say "【收尾】"

  if [ "$DRY_RUN" = 1 ]; then
    say "  [dry-run] 跳过实际收尾"
    return 0
  fi
  if [ "$CHECK_ONLY" = 1 ]; then
    say "  [--check] 本次没有启动任何服务，跳过收尾"
    return 0
  fi

  # 1) 发布器（本次启动的；也兜底清掉任何残留）
  local pids i
  if [ -n "$PUB_PID" ] && kill -0 "$PUB_PID" 2>/dev/null; then
    say "  停止发布器 (PID $PUB_PID)"
    kill -INT "$PUB_PID" 2>/dev/null || true
    for i in 1 2 3 4 5 6; do
      if ! kill -0 "$PUB_PID" 2>/dev/null; then break; fi
      sleep 0.4
    done
    if kill -0 "$PUB_PID" 2>/dev/null; then kill -9 "$PUB_PID" 2>/dev/null || true; fi
  fi
  pids="$(pgrep -f "[v]r_data_publisher" 2>/dev/null || true)"
  if [ -n "$pids" ]; then
    say "  停止残留发布器: $pids"
    printf '%s\n' "$pids" | xargs -r kill -9 2>/dev/null || true
  fi

  # 2) Ray（连同它的 worker 进程）
  say "  停止 Ray 集群"
  "$RAY" stop --force >/dev/null 2>&1 || true

  # 3) PC Service
  if pgrep -f "[R]oboticsServiceProcess" >/dev/null 2>&1; then
    say "  停止 PC Service"
    pkill -f "[R]oboticsServiceProcess" 2>/dev/null || true
    sleep 1
    pkill -9 -f "[R]oboticsServiceProcess" 2>/dev/null || true
  fi

  # 4) 汇总
  hr
  ok "收尾完成"
  say "  残留检查（都应为 0）："
  printf '    %-14s %s\n' "采集"        "$(pgrep -f "[c]ollect_real_data" 2>/dev/null | wc -l || true)"
  printf '    %-14s %s\n' "Ray"         "$(pgrep -f "[r]aylet|[g]cs_server" 2>/dev/null | wc -l || true)"
  printf '    %-14s %s\n' "发布器"      "$(pgrep -f "[v]r_data_publisher" 2>/dev/null | wc -l || true)"
  printf '    %-14s %s\n' "PC Service"  "$(pgrep -f "[R]oboticsServiceProcess" 2>/dev/null | wc -l || true)"
  say ""
  say "  机械臂会停在当前位置，不会自动归位。"
  say "  会话日志：$SESSION_LOG"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# -----------------------------------------------------------------------------
# 阶段 0：预检
# -----------------------------------------------------------------------------
phase_preflight() {
  hr; say "【0/6】预检"

  local missing=0 p
  for p in "$PY" "$RAY" "$WRAPPER" "$SYNC_TOOL" "$PROBE" \
           "$PICO_VENV/bin/activate" "$PICO_DIR/$PICO_CFG_NAME" \
           "$PC_SERVICE_DIR/runService.sh"; do
    if [ -e "$p" ]; then
      say "  ✅ $p"
    else
      err "  ❌ 缺失：$p"
      missing=1
    fi
  done
  if [ "$missing" = 1 ]; then
    die "关键路径缺失，请先按文档《从零搭建流程》处理。"
  fi

  local camera missing_camera=0
  say ""
  say "  相机视频节点："
  for camera in \
    "usb-Technologies__Inc._ZED_2i_OV0001-video-index0" \
    "usb-Technologies__Inc._ZED-M-video-index0"; do
    if [ -e "/dev/v4l/by-id/$camera" ]; then
      say "  ✅ /dev/v4l/by-id/$camera → $(readlink -f "/dev/v4l/by-id/$camera")"
    else
      err "  ❌ 缺失：/dev/v4l/by-id/$camera"
      missing_camera=1
    fi
  done
  if [ "$missing_camera" = 1 ]; then
    say "  ZED 相机只有 HID、没有 video 节点时，通常未接入 USB 3.0 数据路径。"
    say "  请将缺失相机改接直连 USB 3.0 端口（不要使用 USB 2.0 hub），确认节点出现后重试。"
    die "相机视频节点缺失，拒绝在连接 Franka 前启动采集"
  fi

  say ""
  say "  网络接口："
  ip -4 -o addr show scope global 2>/dev/null | awk '{printf "    %-10s %s\n", $2, $4}' \
    | tee -a "$SESSION_LOG" || true

  PC_IP="$(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 \
           | grep '^192\.168\.50\.' | head -1 || true)"
  if [ -z "$PC_IP" ]; then PC_IP="<本机在热点网段的 IP>"; fi
  say "    第 3 步头显里要填的 PC IP：$PC_IP"

  if ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
    say "  ✅ 机械臂 $ROBOT_IP 可达"
  else
    warn "  ⚠️ 机械臂 $ROBOT_IP ping 不通（若还没上电可忽略，第 2 步会重试）"
  fi

  if [ "$CHECK_ONLY" = 0 ] && [ "$DRY_RUN" = 0 ]; then
    say "  清理残留 Ray 集群"
    "$RAY" stop --force >/dev/null 2>&1 || true
  fi
}

# -----------------------------------------------------------------------------
# 阶段 1：PC Service
# -----------------------------------------------------------------------------
phase_pc_service() {
  hr; say "【1/6】前置服务：XRoboToolkit PC Service"

  if [ "$DRY_RUN" = 1 ]; then say "  [dry-run] 跳过启动"; return 0; fi

  if pgrep -f "[R]oboticsServiceProcess" >/dev/null 2>&1 \
     && ss -ltn 2>/dev/null | grep -q ':63901'; then
    ok "  已在运行并监听 63901 —— 跳过启动"
    return 0
  fi

  say "  启动中（日志：$PCS_LOG）"
  ( cd "$PC_SERVICE_DIR" && bash runService.sh ) >>"$PCS_LOG" 2>&1 || true

  local i
  for i in $(seq 1 30); do
    if ss -ltn 2>/dev/null | grep -q ':63901'; then break; fi
    sleep 0.5
  done

  if ss -ltn 2>/dev/null | grep -q ':63901'; then
    ok "  ✅ 已监听 63901"
  else
    err "  ❌ 启动后 30 秒内没有监听 63901"
    say "  日志尾部："
    tail -8 "$PCS_LOG" 2>/dev/null | sed 's/^/    /' || true
    die "PC Service 启动失败"
  fi
}

# -----------------------------------------------------------------------------
# 阶段 2：机械臂只读检查 + 位姿核对
# -----------------------------------------------------------------------------
phase_arm_check() {
  hr; say "【2/6】机械臂检查（只读，不产生运动）"

  if [ "$DRY_RUN" = 1 ]; then say "  [dry-run] 跳过（不连接机械臂）"; return 0; fi

  local attempt=0 out="" rc=0
  while :; do
    attempt=$((attempt + 1))
    out="$("$PY" "$SYNC_TOOL" 2>&1)"; rc=$?
    if [ "$rc" -eq 0 ]; then break; fi

    err "  连接机械臂失败：$(printf '%s' "$out" | tail -1)"
    say "  请依次确认："
    say "    ① 机械臂已上电"
    say "    ② Franka Desk 已【解锁到可编程模式】"
    say "    ③ Desk 里【FCI 已打开】"
    if [ "$attempt" -ge 3 ]; then
      die "机械臂不可用（已尝试 3 次）。处理完请重新运行本脚本。"
    fi
    pause_enter "在 Desk 里处理好后，按回车重试"
  done

  printf '%s\n' "$out" | sed 's/^/  /' | tee -a "$SESSION_LOG"

  local pose_ok=0 win_ok=0 box_ok=0
  if printf '%s' "$out" | grep -q '位姿=一致'; then pose_ok=1; fi
  if printf '%s' "$out" | grep -q '姿态窗口=居中'; then win_ok=1; fi
  if printf '%s' "$out" | grep -q '位置盒=包含'; then box_ok=1; fi

  # 复位点跑出位置盒 = 硬风险，直接中止：
  #   reset 的插值移动【不经过】安全盒（base.py:271/594），
  #   但 env.step() 每步都把「实测位姿 + 增量」裁到盒边界（base.py:389）。
  #   两者不一致时，即使操作者不动手柄，第一步的目标也会被裁到盒边界，
  #   机械臂会在【无人指令】下自己朝盒边界走（受 franky 限速，约 0.3 m/s）。
  if [ "$box_ok" = 0 ]; then
    err ""
    err "  ❌ 复位点不在位置盒内 —— 拒绝启动采集"
    say "     原因：reset 插值移动不经过安全盒，但 env.step() 每步都会把目标裁到盒边界"
    say "           （base.py:389）。两者不一致时机械臂会【无人指令地】向盒边界移动。"
    say "     处理：把 ee_pose_limit_min/max 的 [:3] 改成覆盖当前复位点的范围，"
    say "           或把机械臂移回位置盒内，然后重新运行本脚本。"
    die "位置盒与复位点不一致，已中止（收尾会停掉已启动的服务）。"
  fi

  if [ "$pose_ok" = 1 ] && [ "$win_ok" = 1 ]; then
    ok "  ✅ 位姿与 reset_ee_pose 一致，且姿态窗口以复位姿态为中心"
    say "     → 启动零位移；末端往各方向可转角度对称。"
    return 0
  fi

  say ""
  if [ "$pose_ok" = 0 ]; then
    warn "  ⚠️ 位姿与配置里的 reset_ee_pose 不一致"
    say "  启动采集时机械臂会【插值移动到 reset_ee_pose】。"
  fi
  if [ "$win_ok" = 0 ]; then
    warn "  ⚠️ 姿态窗口没有以复位姿态为中心（部分方向的可用转角偏小）"
    say "  这是「转手腕末端转不动」的成因之一（文档 7.26），建议一并修正。"
  fi

  if [ "$CHECK_ONLY" = 1 ]; then
    say ""
    say "  (--check 模式：只报告、不修改配置)"
    say "  如需同步位姿并重新居中姿态窗口，请手动运行："
    say "    $PY $SYNC_TOOL --apply"
    return 0
  fi

  if [ ! -t 0 ]; then
    say ""
    warn "  非交互式终端（stdin 不是 tty）：保持原配置，不自动修改。"
    warn "  启动时机械臂会移动到 reset_ee_pose，请确认工作区无人、无障碍物。"
    return 0
  fi

  say ""
  say "    y = 同步：reset_ee_pose ← 当前位姿（启动零位移），"
  say "             并把姿态窗口重新居中到它（只平移，尺寸不变）（推荐）"
  say "    n = 不同步：保持配置 → 启动时机械臂会自动移动过去，姿态窗口可能仍偏心"
  say ""
  say "  注意：选 y 之前，请确认机械臂【当前这个位置】适合当作复位点。"
  printf '  选择 [Y/n]: '

  local ans=""
  if ! read -r ans; then
    say ""
    warn "  读取输入失败，保持原配置（不做任何修改）。"
    return 0
  fi
  case "$ans" in
    n|N)
      warn "  保持原配置：启动时机械臂会移动到 reset_ee_pose，请确认工作区无人、无障碍物。"
      ;;
    *)
      if "$PY" "$SYNC_TOOL" --apply >/dev/null 2>&1; then
        ok "  ✅ 已同步 reset_ee_pose = 当前位姿（启动零位移）"
        say "     并已把姿态窗口【重新居中】到新复位姿态（半宽不变，只平移不放宽）。"
        say "     位置盒 [:3] 未改动 —— 它绑定物理工作台，不随复位点漂移。"
      else
        die "同步失败，详见 $SESSION_LOG"
      fi
      ;;
  esac
}

# -----------------------------------------------------------------------------
# 阶段 3：暂停 —— 让操作者戴头显、站位、完成头显内设置
# -----------------------------------------------------------------------------
phase_headset() {
  hr; say "【3/6】暂停 —— 请完成头显侧配置"
  say ""
  say "  ① 戴上头显，站在机械臂【后面】、与它【同向】（面向工作台）"
  say ""
  say "  ② 头显 app（XRoboToolkit）里设置："
  say "       Network → PC Service → 填  $PC_IP"
  say "                              → Status 显示 WORKING"
  say "       Tracking → Head / Controller / Send  全部打开"
   say "       Switch w/ A Button → 关闭（A 键用于 A+X 复位组合键）"
  say "       PICO Motion Tracker → Mode → None"
  say ""
  say "  ③ 捏住 grip 两秒再松开（顺带确认手柄有反应）"
  say ""
  say "  ④ 标定先别做：扣 trigger 的标定要在 RLinf 起来之后才生效"
  pause_enter "以上都完成后"
}

# -----------------------------------------------------------------------------
# 阶段 4：发布器
# -----------------------------------------------------------------------------
phase_publisher() {
  hr; say "【4/6】启动 VR 数据发布器"

  if [ "$DRY_RUN" = 1 ]; then say "  [dry-run] 跳过启动"; return 0; fi

  # 已有发布器一律重开：它的 socket inode 必须与随后的 RLinf 匹配
  if pgrep -f "[v]r_data_publisher" >/dev/null 2>&1; then
    warn "  检测到已有发布器，先停掉它再重开"
    pkill -f "[v]r_data_publisher" 2>/dev/null || true
    sleep 2
  fi

  : > "$PUB_LOG"
  (
    cd "$PICO_DIR" || exit 1
    # shellcheck disable=SC1090
    source "$PICO_VENV/bin/activate" || exit 1
    export NO_PROXY="127.0.0.1,localhost" no_proxy="127.0.0.1,localhost"
    exec python -u -m vr_data_publisher --config "$PICO_CFG_NAME"
  ) >>"$PUB_LOG" 2>&1 &
  PUB_PID=$!

  say "  发布器 PID=$PUB_PID，日志：$PUB_LOG"
  say "  等待 device found（最多 30 秒）..."

  local i
  for i in $(seq 1 30); do
    if grep -q 'device found' "$PUB_LOG" 2>/dev/null; then
      ok "  ✅ device found"
      return 0
    fi
    if ! kill -0 "$PUB_PID" 2>/dev/null; then
      err "  发布器已退出。日志尾部："
      tail -8 "$PUB_LOG" 2>/dev/null | sed 's/^/    /' || true
      die "发布器启动失败"
    fi
    sleep 1
  done

  err "  30 秒内没有出现 device found"
  say "  这说明头显没连上或没在推流。"
  say "  处理：确认头显已连接（Network → Status = WORKING）且 Tracking → Send 已打开，"
  say "        然后重新运行本脚本（发布器不会自己从设备断连中恢复）。"
  die "发布器未找到设备"
}

# -----------------------------------------------------------------------------
# 阶段 5：数据探针
# -----------------------------------------------------------------------------
phase_probe() {
  hr; say "【5/6】验证 Pico 数据流"

  if [ "$DRY_RUN" = 1 ]; then say "  [dry-run] 跳过"; return 0; fi

  local out="" rc=0 summary="" grip_max=""
  out="$("$PY" "$PROBE" --zmq-addr "$ZMQ_ADDR" --duration 6 2>&1)"; rc=$?
  printf '%s\n' "$out" | sed 's/^/  /' | tee -a "$SESSION_LOG"

  summary="$(printf '%s' "$out" | grep -o '^PROBE_RESULT .*' | head -1 || true)"
  if [ "$rc" -ne 0 ]; then
    err ""
    err "  ❌ 数据未通过检查 —— 现在启动采集，机械臂按住 grip 也不会动。"
    say "  常见原因与处置："
    say "    · 一条消息都没有  → 发布器没起来"
    say "    · 数值全为 0      → 发布器启动时没找到设备 → 重跑本脚本"
    say "    · 速率过低        → 头显刚连上还没稳定，等几秒重跑本脚本"
    say "  详见文档《故障排查》。"
    die "Pico 数据检查未通过"
  fi

  grip_max="$(printf '%s' "$summary" | tr ' ' '\n' | grep '^max_grip=' | cut -d= -f2 || true)"
  ok "  ✅ 数据在流动且非 0"
  if [ -n "$grip_max" ] && awk -v g="$grip_max" 'BEGIN{exit !(g>=0.9)}'; then
    ok "  ✅ 探针期间捕捉到 grip 按下 —— 手柄通道确认可用"
  else
    warn "  ℹ️ 探针期间没捕捉到 grip 按下（可能你没捏）"
    say "     启动后第一件事：按住 grip 看机械臂是否跟随。"
  fi
}

# -----------------------------------------------------------------------------
# 阶段 6：启动采集（前台，Ctrl-C 即停）
# -----------------------------------------------------------------------------
phase_collect() {
  hr; say "【6/6】启动 RLinf 采集"

  local extra=""
  if [ "${#HYDRA_ARGS[@]}" -gt 0 ]; then extra="${HYDRA_ARGS[*]}"; fi
  say "  命令：bash scripts/run_collect_pico.sh $extra"
  say "  本次录制超时阈值：${RLINF_KEYBOARD_TIMEOUT_STEPS:-3000} 步"
  if [ -n "${RLINF_KEYBOARD_TIMEOUT_STEPS:-}" ]; then
    warn "  ⚠️ 临时超时验证模式：左摇杆按下开始后，不按 Y，等待该步数自动收尾。"
  fi
  say "  出现进度条即启动成功（约 40~60 秒）。"
  say "  ★ 按 Ctrl-C 可随时停止，本脚本会自动完成收尾。"
  hr

  if [ "$DRY_RUN" = 1 ]; then say "  [dry-run] 不真正启动采集"; return 0; fi

  "$RAY" stop --force >/dev/null 2>&1 || true
  if [ "${#HYDRA_ARGS[@]}" -gt 0 ]; then
    bash "$WRAPPER" "${HYDRA_ARGS[@]}"
  else
    bash "$WRAPPER"
  fi
}

# -----------------------------------------------------------------------------
# 主流程
# -----------------------------------------------------------------------------
main() {
  hr
  say "  Pico × Franka 采集会话 —— 一键启动"
  say "  工作目录：$WORK_ROOT"
  say "  会话日志：$SESSION_LOG"
  if [ "$DRY_RUN" = 1 ]; then warn "  模式：--dry-run（只打印，不真启动）"; fi
  if [ "$CHECK_ONLY" = 1 ]; then warn "  模式：--check（只预检，不启动服务/不启动采集）"; fi
  hr

  phase_preflight

  if [ "$CHECK_ONLY" = 1 ]; then
    phase_arm_check
    hr
    ok "预检完成（--check 模式，未启动任何服务、未启动采集）"
    exit 0
  fi

  phase_pc_service
  phase_arm_check
  phase_headset
  phase_publisher
  phase_probe

  hr
  warn "  ⚠️ 即将启动采集：机械臂会移动到 reset_ee_pose。"
  warn "     请确认工作区已清空、急停按钮在手边。"
  hr

  phase_collect

  hr
  say "采集进程已结束。"
}

main "$@"
