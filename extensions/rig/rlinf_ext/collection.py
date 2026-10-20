# Copyright 2026 The RLinf-Franka Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Collection-time extensions for this rig: Pico episode control and rich PICO info.

Both behaviours are reached through RLinf's official out-of-tree hook
(``RLINF_EXT_MODULE``), so RLinf's own tree stays unmodified.

``pico_start_end_timeout``
    Pico button episode control plus a recorded-span ceiling.

Rich PICO info
    ``Pico._reported`` is the only place a PICO reading turns into ``info``, and
    ``CollectEpisode`` copies ``info`` into the episode buffer verbatim. Wrapping it
    keeps the controller's own motion (``position_delta`` / ``rotation_delta``), which
    the stock wrapper drops, so the pickle retains what the operator actually did.
"""

from __future__ import annotations

import os
from typing import Any, Mapping

import gymnasium as gym
import numpy as np

from rlinf.utils.logging import get_logger

_logger = get_logger()

#: Episode-control mode name registered into ``KEYBOARD_MODES``.
KEYBOARD_MODE_NAME = "pico_start_end_timeout"

#: Optional per-process override used only for a bounded timeout verification run.
TIMEOUT_STEPS_ENV_VAR = "RLINF_KEYBOARD_TIMEOUT_STEPS"

#: Reading fields copied into ``info`` as ``pico_*`` by the PICO wrapper.
REPORTED_DELTAS = ("position_delta", "rotation_delta")


class PicoStartEndTimeout(gym.Wrapper):
    """Control one collection episode from approved Pico button events.

    The outer ``RealWorldEnv.max_episode_steps`` must exceed
    :pyattr:`MAX_RECORD_STEPS` so its watchdog owns a recording timeout.
    """

    #: Recorded steps after which an episode is closed and discarded.
    MAX_RECORD_STEPS = 3000

    def __init__(self, env: Any) -> None:
        super().__init__(env)
        self._max_record_steps = self._timeout_steps_from_env()
        source = TIMEOUT_STEPS_ENV_VAR if os.getenv(TIMEOUT_STEPS_ENV_VAR) else "default"
        _logger.info(
            "PicoStartEndTimeout recording ceiling: %d steps (source=%s).",
            self._max_record_steps,
            source,
        )
        self._recorded_steps = 0
        self._counting = False
        self._recording = False

    @classmethod
    def _timeout_steps_from_env(cls) -> int:
        """Read an explicit per-run ceiling without changing the safe default."""
        value = os.getenv(TIMEOUT_STEPS_ENV_VAR)
        if value is None:
            return cls.MAX_RECORD_STEPS
        try:
            steps = int(value)
        except ValueError as error:
            raise ValueError(
                f"{TIMEOUT_STEPS_ENV_VAR} must be a positive integer, got {value!r}"
            ) from error
        if steps <= 0:
            raise ValueError(
                f"{TIMEOUT_STEPS_ENV_VAR} must be positive, got {steps}"
            )
        return steps

    def begin_episode(self) -> None:
        """Clear state before the next environment episode."""
        self._recorded_steps = 0
        self._counting = False
        self._recording = False

    def reset(self, **kwargs: Any) -> tuple[Any, dict[str, Any]]:
        """Clear recording state after an environment reset completes."""
        self.begin_episode()
        return self.env.reset(**kwargs)

    def step(self, action: Any) -> tuple[Any, Any, bool, bool, dict[str, Any]]:
        """Apply Pico recording events after teleoperation has read the packet."""
        obs, reward, terminated, truncated, info = self.env.step(action)
        event = info.get("pico_control_event")
        record_reset = False
        event_status = None
        recording_before = self._recording

        if event == "record_start" and not self._recording:
            self._recording = True
            record_reset = True
            self._recorded_steps = 0
            self._counting = True
            event_status = "accepted"
        elif event == "record_start":
            event_status = "ignored_already_recording"
        elif event == "record_success" and self._recording:
            reward = 1.0
            terminated = True
            self._counting = False
            event_status = "accepted"
        elif event == "record_success":
            event_status = "ignored_not_recording"
        elif event == "record_discard" and self._recording:
            self._recording = False
            record_reset = True
            self._recorded_steps = 0
            self._counting = False
            # End the environment episode so its normal reset path returns the
            # arm home after the discarded buffer has been cleared.
            terminated = True
            event_status = "accepted"
        elif event == "record_discard":
            event_status = "ignored_not_recording"
        elif event == "robot_reset":
            self._recording = False
            record_reset = True
            self._recorded_steps = 0
            self._counting = False
            terminated = True
            event_status = "accepted"

        if self._counting and not record_reset and not terminated:
            self._recorded_steps += 1
            if self._recorded_steps >= self._max_record_steps:
                self._counting = False
                self._recorded_steps = 0
                terminated = True
                truncated = True
                event = "timeout"
                # Keep the terminal frame in the active buffer for timeout
                # provenance before the environment's auto-reset clears it.
                info["keyboard_phase"] = "rec"
                info["pre_record"] = False
                _logger.warning(
                    "PicoStartEndTimeout: recorded span reached %d steps; "
                    "closing the episode without a success reward.",
                    self._max_record_steps,
                )

        info["pre_record"] = not self._recording
        info["record_reset"] = record_reset
        info["keyboard_phase"] = "rec" if self._recording else "pre"
        info["keyboard_event"] = event
        if event is not None:
            info["pico_event_status"] = event_status or "forwarded"
            info["pico_recording_before"] = recording_before
            _logger.info(
                "Pico event=%s status=%s recording=%s record_reset=%s",
                event,
                info["pico_event_status"],
                f"{recording_before}->{self._recording}",
                record_reset,
            )
        info["segment_advance"] = False
        return obs, reward, terminated, truncated, info


def register_pico_timeout_mode() -> None:
    """Register the Pico collection-control mode."""
    from rlinf.envs.real.wrappers import KEYBOARD_MODES

    if KEYBOARD_MODE_NAME in KEYBOARD_MODES:
        _logger.info("Keyboard mode %r already registered.", KEYBOARD_MODE_NAME)
        return

    KEYBOARD_MODES[KEYBOARD_MODE_NAME] = PicoStartEndTimeout
    _logger.info(
        "Registered Pico episode mode %r with a %d-step recorded-span ceiling.",
        KEYBOARD_MODE_NAME,
        PicoStartEndTimeout.MAX_RECORD_STEPS,
    )


def install_rich_pico_info() -> None:
    """Add the controller's own motion to every PICO ``info`` dict.

    ``Pico._reported`` is a staticmethod reached through the class, so replacing the
    class attribute covers both ``PicoDelta`` and ``PicoTcp``. Re-installing is a
    no-op, which keeps repeated ``register()`` calls safe.
    """
    from rlinf.robotics.parts.teleop.pico import Pico

    existing = Pico.__dict__.get("_reported")
    if existing is None:
        raise RuntimeError(
            "Pico._reported is not defined on Pico itself; the rich-info wrapper "
            "cannot be installed. RLinf moved the attribute to a base class — "
            "update rlinf_ext.collection to wrap the new owner."
        )
    # A staticmethod does not forward attribute access to its function, so the
    # installed marker has to be read off ``__func__``.
    if getattr(getattr(existing, "__func__", None), "_rlinf_franka_rich", False):
        _logger.info("Rich PICO info is already installed.")
        return

    original = existing.__func__

    def _reported(reading: Mapping[str, Any]) -> dict[str, Any]:
        info = original(reading)
        for field in REPORTED_DELTAS:
            value = reading.get(field)
            info[f"pico_{field}"] = (
                np.asarray(value, dtype=np.float32)
                if value is not None
                else np.zeros(3, dtype=np.float32)
            )
        return info

    _reported._rlinf_franka_rich = True
    Pico._reported = staticmethod(_reported)
    _logger.info(
        "Rich PICO info installed; added %s to info.",
        ", ".join(f"pico_{field}" for field in REPORTED_DELTAS),
    )
