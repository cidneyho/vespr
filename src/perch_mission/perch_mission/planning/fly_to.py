"""Phase 1, fly to the position: implementations.

straight: Goto the target, turn to face the given heading, Done once both have finished.
"""
import math

from perch_mission.planning.steps import Done, Goto, Pacer, Wait, YawBy


class Straight:

    def __init__(self, target_north=0.0, target_east=0.0, target_altitude=1.5,
                 target_heading=90.0, cruise_speed=0.5, settle_time=0.5):
        self.target = (target_north, target_east, target_altitude)
        self.heading = target_heading
        self.speed = cruise_speed
        self.pacer = Pacer(settle_time)

    def start(self, situation):
        self.stage = 'goto'

    def next_step(self, situation):
        now = situation.time
        if self.pacer.busy(now):
            return Wait()
        if situation.position is None or situation.heading is None:
            return Wait()
        if self.stage == 'goto':
            self.stage = 'turn'
            distance = math.dist(situation.position, self.target)
            return self.pacer.moved(Goto(*self.target, self.speed), now, distance)
        if self.stage == 'turn':
            self.stage = 'done'
            turn = math.remainder(self.heading - situation.heading, 360.0)  # the short way round
            if abs(turn) > 0.5:
                return self.pacer.moved(YawBy(turn), now)
        return Done()
