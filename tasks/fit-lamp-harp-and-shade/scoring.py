"""Harp retention and final assembly scoring."""

from dataclasses import dataclass

PITCH = .0254/27


@dataclass(frozen=True)
class LegGauge:
    gap: float
    shoulder_overlap: float
    sleeve_cover: float

    @property
    def hooked(self):
        return -.0008 <= self.gap <= .0018 and self.shoulder_overlap >= .0012

    @property
    def retained(self):
        return self.hooked and self.sleeve_cover >= .002


class Assembly:
    def __init__(self):
        self.last_tick = -1
        self.dwell = 0
        self.score = 0.
        self.invalid = False

    def update(self, tick, notches, sleeves, shade, released, valid):
        if tick == self.last_tick:
            return self.score
        self.last_tick = tick
        self.invalid |= not valid
        harp = notches and sleeves
        self.dwell = min(self.dwell+1,20) if harp and shade and released else 0
        self.score = 0. if self.invalid or not harp else 1. if self.dwell == 20 else .5
        return self.score
