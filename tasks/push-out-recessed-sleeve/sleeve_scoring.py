"""Current-state score for sleeve travel, capture and release."""

from dataclasses import dataclass


@dataclass(frozen=True)
class State:
    travel: float = 0.
    rear_projection: float = 0.
    outside: bool = False
    in_tray: bool = False
    empty_bore: bool = False
    rod_parked: bool = False
    plate_stable: bool = False
    released: bool = False
    lost: bool = False


def score(state):
    if state.lost:
        return 0.
    if state.outside and state.in_tray:
        if state.empty_bore and state.rod_parked and state.plate_stable and state.released:
            return 1.
        return .75
    if state.travel >= .010 and state.rear_projection >= .008:
        return .50
    return .25 if state.travel >= .004 else 0.
