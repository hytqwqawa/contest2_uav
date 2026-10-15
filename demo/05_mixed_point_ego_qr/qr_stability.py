"""ROS-independent QR stability and freshness logic for Demo6."""


class QrStability:
    def __init__(self, min_frames=3, freshness_s=1.5):
        if not isinstance(min_frames, int) or min_frames < 1:
            raise ValueError('min_frames must be positive integer')
        self.min_frames = min_frames
        self.freshness_s = freshness_s
        self.candidate = ''
        self.streak = 0
        self.stable = ''
        self.last_seen = None
        self.last_frame = None
        self.ambiguous = False

    def process(self, values, now):
        """Pass QR payloads recognized in one *processed* RGB image frame."""
        values = sorted(set(s for s in values if isinstance(s, str) and s))
        self.last_frame = now
        self.ambiguous = len(values) > 1
        if len(values) != 1:
            self.candidate = ''
            self.streak = 0
            self.stable = ''
            self.last_seen = None
            return
        value = values[0]
        if value == self.candidate:
            self.streak += 1
        else:
            self.candidate = value
            self.streak = 1
        if self.streak >= self.min_frames:
            self.stable = value
            self.last_seen = now
        else:
            self.stable = ''
            self.last_seen = None

    def current(self, now):
        if (self.stable and self.last_seen is not None
                and now - self.last_seen <= self.freshness_s):
            return self.stable
        return ''

    def status(self, now):
        if self.last_frame is None or now - self.last_frame > 2.0:
            return 'NO_CAMERA'
        if self.ambiguous:
            return 'AMBIGUOUS'
        return 'DETECTED' if self.current(now) else 'SEARCHING'
