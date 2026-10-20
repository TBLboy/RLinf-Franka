# Copyright 2026 The RLinf-Franka Authors.
# Licensed under the Apache License, Version 2.0.
"""Accumulate independently timestamped episode streams without resampling."""

from __future__ import annotations

from copy import deepcopy
import os
from typing import Any


class RawEpisode:
    """Keep source samples and loss indicators until the episode is sealed.

    Raw stream boundaries are not clipped: samples delivered in batches around
    the start or end remain available to offline alignment and diagnostics.
    """

    def __init__(self, start_ns: int, periods_ns: dict[str, int]) -> None:
        if isinstance(start_ns, bool) or not isinstance(start_ns, int) or start_ns <= 0:
            raise ValueError("episode start must be a positive monotonic timestamp")
        self.start_ns = start_ns
        self.periods_ns = dict(periods_ns)
        self.streams: dict[str, list[dict[str, Any]]] = {}
        self.losses: dict[str, int] = {}
        self.end_ns: int | None = None
        self.events: list[dict[str, Any]] = []
        self.payload_bytes = 0
        self.max_payload_bytes = int(os.environ.get("RLINF_RAW_EPISODE_MAX_BYTES", 1073741824))
        if self.max_payload_bytes <= 0:
            raise ValueError("RLINF_RAW_EPISODE_MAX_BYTES must be positive")

    def append(self, stream: str, samples: list[dict[str, Any]], *, overflow: bool = False,
               owned: bool = False) -> None:
        """Append paired samples, rejecting malformed and reordered batches."""
        if self.end_ns is not None:
            raise RuntimeError("episode already sealed")
        if overflow:
            self.losses[stream] = self.losses.get(stream, 0) + 1
        def payload_size(value: Any) -> int:
            if isinstance(value, dict):
                return sum(payload_size(item) for item in value.values())
            if isinstance(value, (list, tuple)):
                return sum(payload_size(item) for item in value)
            return int(getattr(value, "nbytes", 0))

        size = payload_size(samples)
        if self.payload_bytes + size > self.max_payload_bytes:
            raise MemoryError(
                f"raw episode exceeds {self.max_payload_bytes} payload bytes; "
                "stop recording sooner or explicitly increase RLINF_RAW_EPISODE_MAX_BYTES"
            )
        existing = self.streams.get(stream, [])
        last = existing[-1]["timestamp_ns"] if existing else -1
        for sample in samples:
            timestamp = sample.get("timestamp_ns")
            if (isinstance(timestamp, bool) or not isinstance(timestamp, int)
                    or timestamp <= last or "value" not in sample):
                raise ValueError(f"{stream}: samples require increasing timestamps and values")
            last = timestamp
        # Snapshot the batch: the caller may reuse or mutate its dicts.
        self.streams.setdefault(stream, []).extend(samples if owned else deepcopy(samples))
        self.payload_bytes += size

    def seal(self, end_ns: int) -> dict[str, Any]:
        """Return original streams with explicit boundaries and loss status."""
        if self.end_ns is not None or isinstance(end_ns, bool) or not isinstance(end_ns, int) or end_ns <= self.start_ns:
            raise ValueError("episode end must be after start and seal may run only once")
        self.end_ns = end_ns
        # Returned by move: the episode is discarded right after sealing, and a
        # full copy of every camera frame would double peak memory.
        return {
            "raw_streams": self.streams,
            "episode_bounds_monotonic_ns": {"start": self.start_ns, "end": end_ns},
            "stream_periods_ns": dict(self.periods_ns),
            "raw_stream_losses": dict(self.losses),
            "raw_events": self.events,
        }
