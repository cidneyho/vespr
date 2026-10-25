"""The Planner protocol, and the table of implementations each phase can be set to.

    planner = make('approach', 'standoff', step_length=0.2)
    planner.start(situation)
    step = planner.next_step(situation)   # repeat until Done or Abort
"""
from typing import Protocol

from vespr_flight.planning import approach, fly_to, search
from vespr_flight.planning.steps import Situation

PLANNERS = {
    'fly_to': {'straight': fly_to.Straight},
    'search': {'turn': search.Turn},
    'approach': {'standoff': approach.Standoff},
}


class Planner(Protocol):

    def start(self, situation: Situation) -> None:
        """Begin a fresh run of this phase."""

    def next_step(self, situation: Situation):
        """The next step for the mission: Goto, MoveBody, YawBy, Wait, Grasp, Done or Abort."""


def make(phase, name, **params):
    """A planner for `phase` ('fly_to', 'search' or 'approach') by implementation name."""
    if phase not in PLANNERS:
        raise ValueError(f'unknown phase {phase!r}: one of {", ".join(PLANNERS)}')
    if name not in PLANNERS[phase]:
        raise ValueError(f'unknown {phase} planner {name!r}: one of {", ".join(PLANNERS[phase])}')
    return PLANNERS[phase][name](**params)
