#!/usr/bin/env python3
"""ROS-independent temporal confirmation for one unambiguous blob per color.

The monitor intentionally suppresses a color when more than one blob of that
color is present: it must not assign one ambiguous world coordinate to several
physical targets. Different colors are tracked independently.
"""

import math

COLORS = ('red', 'green', 'blue')


class ColorStability:
    def __init__(self, stable_frames=3, freshness_s=1.5, max_jump_px=45.0):
        if not isinstance(stable_frames, int) or stable_frames < 1:
            raise ValueError('stable_frames must be a positive integer')
        if not (0 < freshness_s <= 10 and 0 < max_jump_px <= 1000):
            raise ValueError('invalid freshness or movement threshold')
        self.stable_frames = stable_frames
        self.freshness_s = float(freshness_s)
        self.max_jump_px = float(max_jump_px)
        self._states = {
            color: dict(last=None, streak=0, confirmed=None, last_seen=None)
            for color in COLORS
        }

    def process(self, observations, now):
        """Update once per processed RGB frame.

        observations is an iterable of dicts with color and pixel_xy fields.
        Zero or multiple blobs of the same color reset that color's stability.
        Return the latest unambiguous, confirmed observations by color.
        """
        grouped = {color: [] for color in COLORS}
        for obs in observations:
            color = obs.get('color')
            if color in grouped:
                xy = obs.get('pixel_xy')
                if (isinstance(xy, (tuple, list)) and len(xy) == 2
                        and all(math.isfinite(float(v)) for v in xy)):
                    grouped[color].append(obs)
        for color, candidates in grouped.items():
            state = self._states[color]
            if len(candidates) != 1:
                state.update(last=None, streak=0, confirmed=None, last_seen=None)
                continue
            obs = candidates[0]
            xy = [float(v) for v in obs['pixel_xy']]
            previous = state['last']
            close = (previous is not None and
                     math.hypot(xy[0] - previous[0], xy[1] - previous[1])
                     <= self.max_jump_px)
            state['streak'] = state['streak'] + 1 if close else 1
            state['last'] = xy
            if state['streak'] >= self.stable_frames:
                state['confirmed'] = obs
                state['last_seen'] = float(now)
            else:
                state['confirmed'] = None
                state['last_seen'] = None
        return self.current(now)

    def current(self, now):
        """Confirmed colors with fresh observations (nothing stale is reused)."""
        out = {}
        for color, state in self._states.items():
            if (state['confirmed'] is not None and state['last_seen'] is not None
                    and 0 <= float(now) - state['last_seen'] <= self.freshness_s):
                out[color] = state['confirmed']
        return out
