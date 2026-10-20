"""No-hardware checks for the Franka/Pico Release adaptation layer."""

from __future__ import annotations

import subprocess
from pathlib import Path



ROOT = Path(__file__).resolve().parents[2]


def test_release_adaptation_paths_exist() -> None:
    required = [
        "rlinf",
        "extensions/rig/rlinf_ext",
        "pico_bridge/vr_data_publisher.py",
        "release_configs/embodiment/realworld_collect_data_pico_single_franka.yaml",
        "scripts/run_pico_session.sh",
        "scripts/check_collection_run.py",
        "scripts/post_process_to_lerobot.py",
        "docs/franka_pico_release/upstream-sync.md",
        "licenses/RLinf-APACHE-2.0.txt",
    ]
    missing = [path for path in required if not (ROOT / path).exists()]
    assert not missing


def test_release_scripts_have_valid_shell_syntax() -> None:
    for script in (ROOT / "scripts").glob("*.sh"):
        result = subprocess.run(["bash", "-n", script], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr


def test_release_python_entry_points_compile() -> None:
    paths = [
        *sorted((ROOT / "scripts").glob("*.py")),
        *sorted((ROOT / "extensions/rig/rlinf_ext").glob("*.py")),
        *sorted((ROOT / "pico_bridge").glob("*.py")),
    ]
    result = subprocess.run(
        ["python3", "-m", "py_compile", *map(str, paths)], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


def test_release_collector_accepts_external_config_root() -> None:
    collector = (ROOT / "examples/embodiment/collect_data.sh").read_text()
    assert "EMBODIED_CONFIG_PATH" in collector
    assert '"${EMBODIED_CONFIG_PATH}"' in collector


def test_launcher_defaults_to_the_verified_vendor_sdk_environment() -> None:
    launcher = (ROOT / "scripts/run_pico_session.sh").read_text()
    assert "RLINF_PICO_SDK_VENV" in launcher
    assert "pico_software/XRoboToolkit-Teleop-Sample-Python/.venv" in launcher


def test_launcher_requires_both_configured_camera_video_nodes() -> None:
    launcher = (ROOT / "scripts/run_pico_session.sh").read_text()
    assert "ZED_2i_OV0001-video-index0" in launcher
    assert "ZED-M-video-index0" in launcher
    assert "相机视频节点缺失" in launcher


def test_release_config_keeps_control_and_dataset_rate_aligned() -> None:
    config = (
        ROOT / "release_configs/embodiment/realworld_collect_data_pico_single_franka.yaml"
    ).read_text()
    assert "step_frequency: 30.0" in config
    assert "fps: 30" in config
    assert "camera_fps: 30" in config


def test_timestamp_alignment_matches_independent_streams_and_rejects_gaps() -> None:
    import importlib.util
    import pytest

    spec = importlib.util.spec_from_file_location(
        "timestamp_alignment", ROOT / "scripts/timestamp_alignment.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    period = 100_000_000
    streams = {
        "camera": [(0, "first"), (100_000_000, "last")],
        "robot": [(0, 1), (100_000_000, 2)],
    }
    rows = module.align_streams(
        streams, {"camera": period, "robot": period},
        episode_start_ns=0, episode_end_ns=100_000_000, fps=10,
    )
    assert rows == [(0, {"camera": "first", "robot": 1}),
                    (100_000_000, {"camera": "last", "robot": 2})]
    with pytest.raises(module.AlignmentError, match="camera"):
        module.align_streams(
            streams, {"camera": 10_000_000, "robot": period},
            episode_start_ns=0, episode_end_ns=100_000_000, fps=30,
        )
    # The exact third tick, rounded down to nanoseconds, is still in range.
    third_tick = 33_333_333
    rows = module.align_streams(
        {"camera": [(third_tick, "frame"), (100_000_000, "end")]},
        {"camera": 100_000_000},
        episode_start_ns=0, episode_end_ns=100_000_000, fps=30,
    )
    assert rows[0][0] == third_tick


def test_raw_episode_preserves_samples_and_marks_overflow() -> None:
    import importlib.util
    import pytest

    spec = importlib.util.spec_from_file_location(
        "raw_episode", ROOT / "extensions/rig/rlinf_ext/raw_episode.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    episode = module.RawEpisode(100, {"pico": 12_500_000})
    packets = [{"timestamp_ns": 90, "value": {"vendor_ns": 2}},
               {"timestamp_ns": 110, "value": {"vendor_ns": 3}}]
    episode.append("pico", packets, overflow=True)
    packets[0]["value"]["vendor_ns"] = 999
    with pytest.raises(ValueError, match="increasing"):
        episode.append("pico", [{"timestamp_ns": 105, "value": {}}])
    raw = episode.seal(120)
    assert raw["raw_streams"]["pico"][0]["value"]["vendor_ns"] == 2
    assert raw["raw_stream_losses"] == {"pico": 1}
    assert raw["episode_bounds_monotonic_ns"] == {"start": 100, "end": 120}
