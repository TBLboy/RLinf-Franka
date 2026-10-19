# Copyright 2026 The RLinf-Franka Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Collection-time extensions for this rig: keyboard episode control and rich PICO info.

Both behaviours are reached through RLinf's official out-of-tree hook
(``RLINF_EXT_MODULE``), so RLinf's own tree stays unmodified.

``start_end_timeout``
    RLinf's ``start_end`` keyboard wrapper plus a recorded-span ceiling.
    ``KeyboardStartEndWrapper`` forces ``terminated``/``truncated`` to ``False`` on
    every step, so ``max_episode_steps`` cannot end an episode and a forgotten ``c``
    records until the operator notices. The watchdog ends the episode once the
    recorded span reaches :pyattr:`KeyboardStartEndTimeout.MAX_RECORD_STEPS`.

Rich PICO info
    ``Pico._reported`` is the only place a PICO reading turns into ``info``, and
    ``CollectEpisode`` copies ``info`` into the episode buffer verbatim. Wrapping it
    keeps the controller's own motion (``position_delta`` / ``rotation_delta``), which
    the stock wrapper drops, so the pickle retains what the operator actually did.
"""

from __future__ import annotations

import os
from typing import Any, Mapping

import numpy as np

from rlinf.utils.logging import get_logger

_logger = get_logger()

#: Keyboard mode name this module registers into ``KEYBOARD_MODES``.
KEYBOARD_MODE_NAME = "start_end_timeout"

#: Optional per-process override used only for a bounded timeout verification run.
TIMEOUT_STEPS_ENV_VAR = "RLINF_KEYBOARD_TIMEOUT_STEPS"

#: Reading fields copied into ``info`` as ``pico_*`` by the PICO wrapper.
REPORTED_DELTAS = ("position_delta", "rotation_delta")


class KeyboardStartEndTimeout:
    """Mixin adding a recorded-span ceiling to a keyboard episode wrapper.

    Mixed into the ``start_end`` wrapper at registration time so the key handling
    stays RLinf's. The outer ``RealWorldEnv.max_episode_steps`` must exceed
    :pyattr:`MAX_RECORD_STEPS`: it otherwise overwrites an inner truncation before
    this watchdog can observe its own ceiling.
    """

    #: Recorded steps after which an episode is closed and discarded.
    MAX_RECORD_STEPS = 3000

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._max_record_steps = self._timeout_steps_from_env()
        source = TIMEOUT_STEPS_ENV_VAR if os.getenv(TIMEOUT_STEPS_ENV_VAR) else "default"
        _logger.info(
            "KeyboardStartEndTimeout recording ceiling: %d steps (source=%s).",
            self._max_record_steps,
            source,
        )
        self._recorded_steps = 0
        self._counting = False

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
        """Reset the recorded span alongside the wrapper's own episode state."""
        super().begin_episode()
        self._recorded_steps = 0
        self._counting = False

    def step(self, action: Any) -> tuple[Any, Any, bool, bool, dict[str, Any]]:
        """Close the episode when the recorded span reaches the ceiling.

        ``start_end`` keeps ``_recording`` True after ``c`` so the terminal frame is
        saved, so counting follows its own flag: ``a`` arms it, ``c`` and a timeout
        disarm it. Anything before the first ``a`` is ``pre_record`` and never counts.
        """
        obs, reward, terminated, truncated, info = super().step(action)

        if info.get("record_reset"):
            # ``a`` starts or aborts a recording; either way the span restarts.
            self._recorded_steps = 0
            self._counting = bool(self._recording)
        elif info.get("keyboard_event") == "end_success":
            self._recorded_steps = 0
            self._counting = False
        elif self._counting:
            self._recorded_steps += 1
            if self._recorded_steps >= self._max_record_steps:
                self._counting = False
                self._recorded_steps = 0
                terminated = True
                truncated = True
                info["keyboard_event"] = "timeout"
                # Keep the terminal frame in the active buffer. CollectEpisode
                # skips ``pre_record`` transitions; this mirrors start_end's ``c``
                # behaviour and lets the pickle retain timeout + done provenance.
                # ``begin_episode`` clears ``_recording`` on the reset that follows.
                info["keyboard_phase"] = "rec"
                info["pre_record"] = False
                _logger.warning(
                    "KeyboardStartEndTimeout: recorded span reached %d steps; "
                    "closing the episode without a success reward.",
                    self._max_record_steps,
                )

        return obs, reward, terminated, truncated, info


def register_keyboard_timeout_mode() -> None:
    """Register the watchdog keyboard mode under :pyattr:`KEYBOARD_MODE_NAME`."""
    from rlinf.envs.real.wrappers import KEYBOARD_MODES
    from rlinf.envs.real.wrappers.episode.start_end import KeyboardStartEndWrapper

    if KEYBOARD_MODE_NAME in KEYBOARD_MODES:
        _logger.info("Keyboard mode %r already registered.", KEYBOARD_MODE_NAME)
        return

    mode = type(
        "KeyboardStartEndTimeoutWrapper",
        (KeyboardStartEndTimeout, KeyboardStartEndWrapper),
        {"__doc__": KeyboardStartEndTimeout.__doc__},
    )
    KEYBOARD_MODES[KEYBOARD_MODE_NAME] = mode
    _logger.info(
        "Registered keyboard mode %r with a %d-step recorded-span ceiling.",
        KEYBOARD_MODE_NAME,
        mode.MAX_RECORD_STEPS,
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
