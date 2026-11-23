"""Track revocable carrier progress and latch an uncontrolled descent."""

from dataclasses import dataclass

UPPER = .035
LOWER = 0.
MAX_DOWNWARD_SPEED = .10
CLEAR_TRAVEL = .070


@dataclass
class Trial:
    dropped: bool = False
    premature_descent: bool = False
    last_height: float | None = None
    peak_speed: float = 0.

    def sample(self, downward_speed, height, travel, dt):
        downward_speed = float(downward_speed)
        height = float(height)
        dt = float(dt)
        if self.last_height is not None:
            downward_speed = max(downward_speed, (self.last_height-height)/dt)
        self.last_height = height
        self.peak_speed = max(self.peak_speed, downward_speed)
        self.dropped = self.dropped or downward_speed > MAX_DOWNWARD_SPEED
        self.premature_descent = self.premature_descent or (.010 <= float(travel) < CLEAR_TRAVEL and height < UPPER-.003)

    def score(self, height, travel, clear, staged, stopped, released):
        if self.dropped or self.premature_descent:
            return 0.
        if clear and staged:
            if height <= LOWER+.003:
                return (1. if stopped and released else .75)
            return .5
        return .25 if travel >= .010 and abs(height-UPPER) <= .003 else 0.
