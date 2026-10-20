"""Strict offline nearest-neighbour alignment of independently timed streams.

Timestamps are integer nanoseconds in the host monotonic clock domain. Each
sample keeps its payload together with its timestamp until it is selected.
"""

from __future__ import annotations

from bisect import bisect_left
from typing import Any, Mapping, Sequence


class AlignmentError(ValueError):
    """The raw episode cannot produce a complete aligned training episode."""


def align_streams(
    streams: Mapping[str, Sequence[tuple[int, Any]]],
    periods_ns: Mapping[str, int],
    *,
    episode_start_ns: int,
    episode_end_ns: int,
    fps: int = 30,
) -> list[tuple[int, dict[str, Any]]]:
    """Select paired payloads on an integer, fixed-rate shared-coverage grid.

    Grid points are anchored at episode start. Ties choose the earlier sample.
    Raises before returning anything when a required stream is absent, malformed,
    or cannot meet its own half-period tolerance at any grid point.
    """
    if (not streams or isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0
            or isinstance(episode_start_ns, bool) or not isinstance(episode_start_ns, int)
            or isinstance(episode_end_ns, bool) or not isinstance(episode_end_ns, int)
            or episode_start_ns < 0 or episode_end_ns <= episode_start_ns):
        raise AlignmentError("invalid episode boundaries, frame rate, or empty streams")
    prepared: dict[str, tuple[list[int], Sequence[tuple[int, Any]], int]] = {}
    for name, samples in streams.items():
        period = periods_ns.get(name)
        if not samples or not isinstance(period, int) or isinstance(period, bool) or period <= 0:
            raise AlignmentError(f"{name}: missing samples or valid sampling period")
        times = [sample[0] for sample in samples]
        if any(not isinstance(t, int) or isinstance(t, bool) or t < 0 for t in times):
            raise AlignmentError(f"{name}: timestamps must be nonnegative integers")
        if any(a >= b for a, b in zip(times, times[1:])):
            raise AlignmentError(f"{name}: timestamps must be strictly increasing")
        if times[0] - episode_start_ns > period or episode_end_ns - times[-1] > period:
            raise AlignmentError(f"{name}: stream does not cover episode boundaries within one source period")
        prepared[name] = (times, samples, period)

    start = max(episode_start_ns, *(times[0] for times, _, _ in prepared.values()))
    end = min(episode_end_ns, *(times[-1] for times, _, _ in prepared.values()))
    # Integer arithmetic avoids floating-point drift and retains the episode anchor.
    first_index = max(0, ((start - episode_start_ns) * fps) // 1_000_000_000)
    while episode_start_ns + first_index * 1_000_000_000 // fps < start:
        first_index += 1
    result: list[tuple[int, dict[str, Any]]] = []
    index = first_index
    while True:
        grid = episode_start_ns + index * 1_000_000_000 // fps
        if grid > end:
            break
        selected: dict[str, Any] = {}
        for name, (times, samples, period) in prepared.items():
            pos = bisect_left(times, grid)
            candidates = [i for i in (pos - 1, pos) if 0 <= i < len(times)]
            nearest = min(candidates, key=lambda i: (abs(times[i] - grid), times[i]))
            error = abs(times[nearest] - grid)
            if error * 2 > period:
                raise AlignmentError(
                    f"{name}: grid={grid} nearest={times[nearest]} "
                    f"error_ns={error} exceeds half-period of {period} ns"
                )
            selected[name] = samples[nearest][1]
        result.append((grid, selected))
        index += 1
    if not result:
        raise AlignmentError("episode and streams have no common 30 Hz grid point")
    return result
