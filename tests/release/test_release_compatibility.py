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
