"""Phase 2, search (board state 3): implementations.

turn: hover and look; if no branch, turn a step and look again. Done once the branch is seen
in several sightings in a row; Abort after a full turn without one.
"""
from vespr_flight.planning.steps import Abort, Done, Pacer, Wait, YawBy

GAP = 0.5  # s without a new sighting that breaks a run of sightings


class Turn:

    def __init__(self, search_step=30.0, search_dwell=1.0, seen_frames=5, settle_time=0.5):
        self.step = search_step
        self.dwell = search_dwell
        self.seen_frames = seen_frames
        self.pacer = Pacer(settle_time)

    def start(self, situation):
        self.turned = 0.0
        self.look_until = situation.time + self.dwell
        self.run = 0             # sightings in a row
        self.last_sighting = None
        self.pacer.ready_at = situation.time  # only sightings from now on

    def next_step(self, situation):
        now = situation.time
        if self.pacer.busy(now):
            return Wait()
        if self.pacer.fresh(situation) and situation.branch_time != self.last_sighting:
            if self.last_sighting is not None and situation.branch_time - self.last_sighting > GAP:
                self.run = 0
            self.run += 1
            self.last_sighting = situation.branch_time
            if self.run >= self.seen_frames:
                return Done()
        if now < self.look_until or self.run > 0 and now - self.last_sighting <= GAP:
            return Wait()  # still looking, or partway through a run of sightings
        if self.turned + self.step > 360.0:
            return Abort('no branch found')
        self.turned += self.step
        self.run = 0
        turn = self.pacer.moved(YawBy(self.step), now)
        self.look_until = self.pacer.ready_at + self.dwell
        return turn
