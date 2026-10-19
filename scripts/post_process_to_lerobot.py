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
import json
import pickle
import sys
from pathlib import Path
from typing import Any

import numpy as np

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
    parser.add_argument("--fps", type=int, default=10)
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
    """Build the LeRobot frame list for one pickle episode."""
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


def collect_episodes(run_dirs: list[Path]) -> list[tuple[Path, dict[str, Any]]]:
    episodes = []
    for run_dir in run_dirs:
        data_dir = run_dir / "collected_data"
        if not data_dir.is_dir():
            print(f"[警告] {run_dir} 下没有 collected_data/，跳过", file=sys.stderr)
            continue
        for path in sorted(data_dir.glob("*.pkl")):
            with path.open("rb") as handle:
                episodes.append((path, pickle.load(handle)))
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
    for path, data in episodes:
        if args.only_success and not bool(data.get("success", False)):
            skipped.append(path.name)
            continue
        frames = episode_frames(data, args.task, stats)
        if not frames:
            skipped.append(path.name)
            continue
        for key in frames[0]:
            if key.startswith("extra_view_image"):
                view_keys.add(key)
        converted.append((path.name, frames))

    if not converted:
        print("[错误] 没有可导出的 episode", file=sys.stderr)
        return 1

    lengths = [len(f) for _, f in converted]
    sample = converted[0][1][0]
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
    writer.create(
        repo_id=args.repo_id,
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
    )
    for name, frames in converted:
        writer.add_episode(frames)
        print(f"[信息] 已写入 {name}（{len(frames)} 帧）")
    writer.finalize()
    print(f"[信息] 完成。repo_id={args.repo_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
