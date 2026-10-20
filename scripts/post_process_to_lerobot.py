#!/usr/bin/env python3
"""Convert a collection run's pickle episodes into a standard LeRobot dataset.

Stage 2 of the two-stage delivery in ``D-021``: collection keeps everything in
``collected_data/*.pkl`` (including fields LeRobot has no slot for), and this script
exports the subset a standard LeRobot dataset carries.

Field mapping, matching what RLinf's own LeRobot path produces:

===========================  ==================================================
pickle                       LeRobot feature
===========================  ==================================================
``observations[i].main_images``        ``image``
``observations[i].extra_view_images``  ``extra_view_image`` (or ``-N`` for several)
``observations[i].states``             ``state``
``infos[i+1].intervene_action``        ``actions``
``infos[i+1].intervene_flag``          ``intervene_flag``
``observations[i].task_descriptions``  ``task``
``success``                            ``is_success``
last frame                             ``done``
===========================  ==================================================

The pickle stores one more observation than action: ``observations[0]`` is the
post-reset frame, so step ``i`` pairs ``observations[i]`` with ``infos[i+1]``. RLinf's
``_buffer_to_lerobot_ep`` makes the same shift.

``actions`` in the pickle's top-level field is always zero (RLinf records the policy
action there, and teleoperation overrides it inside the env). The real command is
``infos[i]["intervene_action"]``, which is what this script exports.

Usage:
    .venv/bin/python scripts/post_process_to_lerobot.py \
        --run-dir logs/<run> --repo-id local/franka-pico-tomato
"""

from __future__ import annotations

import argparse
import pickle
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
from timestamp_alignment import AlignmentError, align_streams

RELEASE_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = RELEASE_ROOT
sys.path.insert(0, str(REPO_ROOT))

from rlinf.data.storage.lerobot import LeRobotDatasetWriter  # noqa: E402

#: Fields the pickle's top-level ``actions`` list would offer but must not be used.
ZERO_ACTION_WARNING = (
    "collected_data/*.pkl 的顶层 actions 恒为 0（A-004）；"
    "本脚本用 infos[i]['intervene_action']，与 demos/*.pt 一致。"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", required=True, action="append",
                        help="包含 collected_data/ 的采集运行目录；可重复")
    parser.add_argument("--repo-id", required=True, help="LeRobot dataset repo id")
    parser.add_argument("--robot-type", default="panda")
    parser.add_argument("--fps", type=int, default=30, choices=[30])
    parser.add_argument("--task", default=None,
                        help="覆盖任务描述；默认取 pickle 里的 task_descriptions")
    parser.add_argument("--only-success", action="store_true",
                        help="只导出 success=True 的 episode")
    parser.add_argument("--dry-run", action="store_true", help="只检查与统计，不写数据集")
    return parser.parse_args()


def to_numpy(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def as_hwc(image: Any) -> np.ndarray | None:
    """Return one image as uint8 [H, W, C]."""
    array = to_numpy(image)
    if array is None:
        return None
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3:
        raise ValueError(f"图像维度不是 3（{array.shape}）")
    if array.shape[0] in (1, 3) and array.shape[0] < array.shape[-1]:
        array = np.transpose(array, (1, 2, 0))
    if array.dtype != np.uint8:
        array = (array * 255).astype(np.uint8) if array.max() <= 1.0 else array.astype(np.uint8)
    return np.ascontiguousarray(array)


def camera_training_image(value: Any) -> np.ndarray:
    """Reproduce the live camera transforms from an archived UVC frame."""
    if not isinstance(value, dict):
        image = as_hwc(value)
        if image is None:
            raise AlignmentError("相机图像缺失")
        return image
    import cv2

    frame = value.get("frame")
    resolution = value.get("resolution")
    eye = value.get("eye")
    if (not isinstance(frame, np.ndarray) or frame.ndim != 3
            or frame.shape[2] != 3 or frame.dtype != np.uint8
            or not isinstance(resolution, (tuple, list)) or len(resolution) != 2
            or eye not in ("left", "right")):
        raise AlignmentError("原始 UVC 图像或相机变换参数无效")
    width, height = resolution
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise AlignmentError("相机分辨率无效")
    half = frame.shape[1] // 2
    if half == 0:
        raise AlignmentError("原始 UVC 图像宽度无效")
    frame = frame[:, :half] if eye == "left" else frame[:, half:]
    h, w = frame.shape[:2]
    if w / h > width / height:
        new_w = max(1, int(round(h * width / height)))
        left = (w - new_w) // 2
        frame = frame[:, left:left + new_w]
    elif w / h < width / height:
        new_h = max(1, int(round(w * height / width)))
        top = (h - new_h) // 2
        frame = frame[top:top + new_h, :]
    if (frame.shape[1], frame.shape[0]) != (width, height):
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    h, w = frame.shape[:2]
    crop = value.get("crop_region")
    if crop is None:
        size = min(h, w)
        y, x = (h - size) // 2, (w - size) // 2
        frame = frame[y:y + size, x:x + size]
    else:
        if (not isinstance(crop, (tuple, list)) or len(crop) != 4
                or not (0 <= crop[0] < crop[2] <= 1 and 0 <= crop[1] < crop[3] <= 1)):
            raise AlignmentError("相机裁剪参数无效")
        frame = frame[int(h * crop[0]):int(h * crop[2]), int(w * crop[1]):int(w * crop[3])]
    if frame.size == 0:
        raise AlignmentError("相机裁剪结果为空")
    return np.ascontiguousarray(cv2.resize(frame, (128, 128), interpolation=cv2.INTER_LINEAR)[..., ::-1])


def extra_views(image: Any) -> dict[str, np.ndarray]:
    """Expand the env's stacked extra-view array into LeRobot frame keys.

    ``RealEnv._wrap_obs`` stacks every non-main frame along axis 1, so the array
    arrives as ``[N, H, W, C]`` (or ``[N_ENV, N, H, W, C]`` batched). One view is
    named ``extra_view_image``, several become ``extra_view_image-N`` — the same rule
    as ``LeRobotFrame._expand_images``.
    """
    array = to_numpy(image)
    if array is None:
        return {}
    while array.ndim > 4:
        array = array[0]
    if array.ndim == 4 and array.shape[0] == 1 and array.shape[-1] in (1, 3):
        array = array[0] if array.shape[1] in (1, 3) else array
    if array.ndim == 3:
        return {"extra_view_image": as_hwc(array)}
    if array.ndim != 4:
        raise ValueError(f"extra_view_images 维度不是 3/4（{array.shape}）")
    # Accept both [N, H, W, C] and [H, N, W, C]; keep the axis holding 1 or 3 channels.
    if array.shape[-1] in (1, 3):
        views = array
    elif array.shape[1] in (1, 3):
        views = np.transpose(array, (1, 0, 2, 3))
    else:
        raise ValueError(f"extra_view_images 通道轴无法识别（{array.shape}）")
    if views.shape[0] == 1:
        return {"extra_view_image": as_hwc(views[0])}
    return {f"extra_view_image-{i}": as_hwc(v) for i, v in enumerate(views)}


def episode_frames(data: dict[str, Any], task_override: str | None,
                   stats: dict[str, int] | None = None) -> list[dict[str, Any]]:
    """Align independently timed raw streams before building training frames."""
    raw = data.get("raw_streams")
    boundaries = data.get("episode_bounds_monotonic_ns")
    periods = data.get("stream_periods_ns")
    required = ("global", "wrist", "robot", "gripper", "action")
    if not isinstance(raw, dict) or not isinstance(boundaries, dict) or not isinstance(periods, dict):
        raise AlignmentError("缺少原始独立时间流、episode 边界或采样周期；不能按索引伪造对齐")
    losses = data.get("raw_stream_losses", {})
    if losses:
        raise AlignmentError(f"原始流缓冲溢出或丢失：{losses}")
    streams = {}
    for name in required:
        samples = raw.get(name)
        if not isinstance(samples, list) or not samples:
            raise AlignmentError(f"{name}: 缺少必需原始数据流")
        try:
            streams[name] = [(s["timestamp_ns"], s["value"]) for s in samples]
        except (KeyError, TypeError, ValueError) as exc:
            raise AlignmentError(f"{name}: 原始样本必须包含 timestamp_ns 和 value") from exc
    try:
        rows = align_streams(
            streams, periods, episode_start_ns=boundaries["start"],
            episode_end_ns=boundaries["end"], fps=30,
        )
    except (KeyError, TypeError, ValueError) as exc:
        if isinstance(exc, AlignmentError):
            raise
        raise AlignmentError("episode 边界格式无效") from exc

    frames = []
    for i, (_, selected) in enumerate(rows):
        robot = selected["robot"]
        action = selected["action"]
        if not isinstance(robot, dict) or not isinstance(robot.get("state_fields"), dict) or not isinstance(action, dict) or action.get("action") is None:
            raise AlignmentError(f"grid={rows[i][0]}: robot state 或 post-guard action 缺失")
        fields = dict(robot["state_fields"])
        segment_id = action.get("segment_id", 0)
        if isinstance(segment_id, bool) or not isinstance(segment_id, int) or not 0 <= segment_id <= 255:
            raise AlignmentError("segment_id must be an integer in [0, 255]")
        expected = {
            "arm_joint_position": 7,
            "arm_joint_torque": 7,
            "arm_joint_velocity": 7,
            "tcp_force": 3,
            "tcp_pose": 6,
            "tcp_torque": 3,
            "tcp_vel": 6,
        }
        if set(fields) != set(expected):
            raise AlignmentError(f"grid={rows[i][0]}: robot 状态字段不完整或未知：{sorted(fields)}")
        for name, size in expected.items():
            value = to_numpy(fields[name])
            if value is None or value.size != size or not np.isfinite(value).all():
                raise AlignmentError(f"grid={rows[i][0]}: robot.{name} 缺失、维度错误或非有限值")
        gripper = to_numpy(selected["gripper"])
        command = to_numpy(action["action"])
        if gripper is None or gripper.size != 1 or not np.isfinite(gripper).all():
            raise AlignmentError(f"grid={rows[i][0]}: gripper 状态无效")
        if command is None or command.size != 7 or not np.isfinite(command).all():
            raise AlignmentError(f"grid={rows[i][0]}: post-guard action 无效")
        fields["gripper_position"] = selected["gripper"]
        state = np.concatenate([to_numpy(fields[key]).reshape(-1) for key in sorted(fields)])
        image, wrist = camera_training_image(selected["global"]), camera_training_image(selected["wrist"])
        if image is None or wrist is None:
            raise AlignmentError(f"grid={rows[i][0]}: 相机图像缺失")
        frames.append({
            "image": image,
            "extra_view_image": wrist,
            "state": state.astype(np.float32),
            "actions": command.reshape(-1).astype(np.float32),
            "task": task_override or str(data.get("task_description", "unknown task")),
            "is_success": np.array([bool(data.get("success", False))], dtype=bool),
            "done": np.array([i == len(rows) - 1], dtype=bool),
            "segment_id": np.array([segment_id], dtype=np.uint8),
            "intervene_flag": np.array([bool(action.get("intervene_flag", False))], dtype=bool),
        })
    return frames


def legacy_episode_frames(data: dict[str, Any], task_override: str | None,
                          stats: dict[str, int] | None = None) -> list[dict[str, Any]]:
    """Historical index-based conversion, not used by the strict CLI."""
    observations = data["observations"]
    infos = data["infos"]
    steps = min(len(observations) - 1, len(infos) - 1)
    if steps <= 0:
        return []

    # Infer the action width from the data instead of assuming seven; a wrong constant
    # would silently emit a channel that does not match ``action_dim``.
    action_dim = 7
    for info in infos:
        if isinstance(info, dict) and info.get("intervene_action") is not None:
            action_dim = int(np.asarray(info["intervene_action"]).reshape(-1).size)
            break

    success = bool(data.get("success", False))
    segment_ids = data.get("segment_ids")
    if segment_ids is None:
        if stats is not None:
            stats["missing_segment_ids"] = stats.get("missing_segment_ids", 0) + 1
        segment_ids = [0] * steps
    elif len(segment_ids) != steps:
        raise ValueError(
            f"segment_ids 长度 {len(segment_ids)} 与动作步数 {steps} 不一致"
        )
    frames: list[dict[str, Any]] = []
    for i in range(steps):
        obs = observations[i]
        info = infos[i + 1] if i + 1 < len(infos) else {}

        action = to_numpy(info.get("intervene_action"))
        if action is None:
            # ``intervene_action`` is normally always present — ``RealWorldEnv.step``
            # fills a zero vector when the operator is not driving — so a missing key
            # means the buffer came from a different producer.
            if stats is not None:
                stats["missing_key"] = stats.get("missing_key", 0) + 1
            action = np.zeros(action_dim, dtype=np.float32)
        action = action.reshape(-1)
        if stats is not None and not np.any(action):
            stats["zero_action"] = stats.get("zero_action", 0) + 1

        frame: dict[str, Any] = {
            "image": as_hwc(obs["main_images"]),
            "state": to_numpy(obs["states"]).reshape(-1).astype(np.float32),
            "actions": action.astype(np.float32),
            "task": task_override or str(obs.get("task_descriptions", "unknown task")),
            "is_success": np.array([success], dtype=bool),
            "done": np.array([i == steps - 1], dtype=bool),
            "segment_id": np.array([segment_ids[i]], dtype=np.uint8),
        }

        flag = to_numpy(info.get("intervene_flag"))
        frame["intervene_flag"] = np.array(
            [bool(flag.reshape(-1).any())] if flag is not None else [False], dtype=bool
        )

        frame.update(extra_views(obs.get("extra_view_images")))
        frames.append(frame)

    return frames


def collect_episodes(run_dirs: list[Path]) -> list[Path]:
    episodes = []
    for run_dir in run_dirs:
        data_dir = run_dir / "collected_data"
        if not data_dir.is_dir():
            print(f"[警告] {run_dir} 下没有 collected_data/，跳过", file=sys.stderr)
            continue
        for path in sorted(data_dir.glob("*.pkl")):
            episodes.append(path)
    return episodes


def main() -> int:
    args = parse_args()
    run_dirs = [Path(p).resolve() for p in args.run_dir]
    episodes = collect_episodes(run_dirs)
    if not episodes:
        print("[错误] 没有找到任何 .pkl", file=sys.stderr)
        return 1

    print(f"[信息] {ZERO_ACTION_WARNING}")
    print(f"[信息] 找到 {len(episodes)} 条 episode")

    converted, skipped, view_keys = [], [], set()
    stats: dict[str, int] = {}
    for path in episodes:
        with path.open("rb") as handle:
            data = pickle.load(handle)
        if args.only_success and not bool(data.get("success", False)):
            skipped.append(path.name)
            continue
        try:
            frames = episode_frames(data, args.task, stats)
        except AlignmentError as exc:
            print(f"[错误] {path}: 时间对齐失败：{exc}", file=sys.stderr)
            return 1
        if not frames:
            skipped.append(path.name)
            continue
        for key in frames[0]:
            if key.startswith("extra_view_image"):
                view_keys.add(key)
        converted.append((path, len(frames)))
        sample = frames[0]
        del data, frames

    if not converted:
        print("[错误] 没有可导出的 episode", file=sys.stderr)
        return 1

    lengths = [length for _, length in converted]
    print(f"[信息] 可导出 {len(converted)} 条，长度 {min(lengths)}~{max(lengths)} 步，"
          f"共 {sum(lengths)} 帧")
    print(f"[信息] 主图 {sample['image'].shape}  state {sample['state'].shape}  "
          f"actions {sample['actions'].shape}  视角 {sorted(view_keys) or ['image']}")
    if skipped:
        print(f"[信息] 跳过 {len(skipped)} 条: {skipped}")
    total_frames = sum(lengths)
    zero = stats.get("zero_action", 0)
    if zero == total_frames:
        print(f"[警告] 全部 {total_frames} 帧的动作都是零 —— 录制窗口内没有遥操作，"
              f"这条数据集对训练没有价值")
    elif zero:
        print(f"[信息] {zero}/{total_frames} 帧动作为零（操作者未握持或未移动）")
    if stats.get("missing_key"):
        print(f"[警告] {stats['missing_key']} 帧缺少 intervene_action 键（已填零）")
    missing_segments = stats.get("missing_segment_ids", 0)
    if missing_segments:
        print(f"[警告] {missing_segments} 条历史 pickle 缺少 segment_ids，"
              "LeRobot 已以 segment_id=0 兼容导出；重新采集后才会保留 b 分段。")

    if args.dry_run:
        print("[信息] --dry-run：未写数据集")
        return 0

    view_shapes = {k: tuple(sample[k].shape) for k in sorted(view_keys)}
    writer = LeRobotDatasetWriter()
    from rlinf.data.storage.lerobot.paths import default_hf_lerobot_home

    destination = (default_hf_lerobot_home() / args.repo_id).resolve()
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite dataset: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging_parent = Path(tempfile.mkdtemp(prefix=".lerobot-partial-", dir=destination.parent))
    staging = staging_parent / "dataset"
    try:
        def load_frames():
            for path, _ in converted:
                with path.open("rb") as handle:
                    data = pickle.load(handle)
                frames = episode_frames(data, args.task)
                del data
                yield path.name, frames

        _write_dataset(writer, args, sample, view_shapes, load_frames(), staging)
        staging.rename(destination)
    finally:
        if writer.dataset is not None:
            image_writer = getattr(writer.dataset, "image_writer", None)
            if image_writer is not None:
                image_writer.stop()
        shutil.rmtree(staging_parent)
    print(f"[信息] 完成。repo_id={args.repo_id}")
    return 0


def _write_dataset(writer, args, sample, view_shapes, converted, staging):
    writer.create(
        repo_id=str(staging),
        robot_type=args.robot_type,
        fps=args.fps,
        image_shape=tuple(sample["image"].shape),
        state_dim=int(sample["state"].shape[0]),
        action_dim=int(sample["actions"].shape[0]),
        has_image=True,
        extra_view_image_keys=view_shapes or None,
        has_intervene_flag=True,
        # Every converted frame now has a segment_id: real values for newly
        # collected pickle, explicit zero compatibility values for old pickle.
        has_segment_id=True,
        image_writer_processes=0,
        image_writer_threads=2,
    )
    for name, frames in converted:
        writer.add_episode(frames)
        print(f"[信息] 已写入 {name}（{len(frames)} 帧）")
    writer.finalize()


if __name__ == "__main__":
    raise SystemExit(main())
