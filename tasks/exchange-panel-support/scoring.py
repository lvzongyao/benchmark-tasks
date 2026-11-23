"""Evaluate the support swap and invalidate a transient panel drop."""


class Exchange:
    def __init__(self):
        self.invalid = False
        self.last_tick = -1
        self.score = 0.
        self.minimum = 0.
        self.maximum = 0.

    def update(self, tick, height_error, withdrawn, supported, staged, new_support, released, valid):
        if tick == self.last_tick:
            return self.score
        self.last_tick = tick
        self.minimum = min(self.minimum, height_error)
        self.maximum = max(self.maximum, height_error)
        self.invalid |= not valid or height_error < -.003 or height_error > .005
        if self.invalid or withdrawn < .010 or not supported:
            self.score = 0.
        elif not staged:
            self.score = .25
        elif not new_support:
            self.score = .50
        else:
            self.score = 1. if released else .75
        return self.score
