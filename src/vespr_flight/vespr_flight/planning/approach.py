"""Phase 3, approach and perch (board states 4 to 7): implementations.

standoff: step to a standoff point squared up to the branch, from which the camera still sees
it, turning on the way so the branch runs across the nose (never so far that it leaves the
view). Then one blind move from there to the capture point, and Grasp.
"""
import math

from vespr_flight.planning.steps import Abort, Done, Grasp, MoveBody, Pacer, Wait, YawBy, rotate


class Standoff:

    def __init__(self, capture_x=0.0, capture_y=0.0, capture_z=0.30,
                 standoff_back=1.0, standoff_down=0.0, approach_speed=0.2, final_speed=0.1,
                 step_length=0.3, yaw_tolerance=5.0, standoff_tolerance=0.05,
                 keep_in_view=25.0, lost_timeout=2.0, max_range=3.0, settle_time=0.5):
        self.capture = (capture_x, capture_y, capture_z)
        # Where the branch is in base_link with the drone at the standoff point
        self.standoff = (capture_x + standoff_back, capture_y, capture_z + standoff_down)
        self.approach_speed = approach_speed
        self.final_speed = final_speed
        self.step_length = step_length
        self.yaw_tolerance = yaw_tolerance
        self.standoff_tolerance = standoff_tolerance
        self.keep_in_view = keep_in_view  # deg, most a turn may leave the branch off the nose
        self.lost_timeout = lost_timeout
        self.max_range = max_range
        self.pacer = Pacer(settle_time)

    def start(self, situation):
        self.stage = 'approach'
        self.started = False
        self.pacer.ready_at = situation.time  # only sightings from now on

    def next_step(self, situation):
        now = situation.time
        if self.pacer.busy(now):
            return Wait()  # including the final move: grasp only once it has ended
        if self.stage == 'grasp':
            self.stage = 'done'
            return Grasp()
        if self.stage == 'done':
            return Done()
        if not self.pacer.fresh(situation):
            if now - self.pacer.ready_at > self.lost_timeout:
                return Abort('lost the branch')
            return Wait()

        (x, y, z), q = situation.branch
        if not self.started:
            self.started = True
            if math.dist((0, 0, 0), (x, y, z)) > self.max_range:
                return Abort(f'branch too far away: {math.dist((0, 0, 0), (x, y, z)):.1f} m')

        # The heading to end up on: the branch's x axis (the detector points it left) straight
        # across the nose. As an anticlockwise turn from now, deg
        ax, ay, _ = rotate(q, (1.0, 0.0, 0.0))
        turn = math.degrees(math.atan2(ay, ax)) - 90.0

        # The standoff point squared up to the branch, in base_link now: where the drone's body
        # should be so the branch sits at self.standoff once the drone has turned by `turn`
        a = math.radians(turn)
        sx, sy, sz = self.standoff
        to_go = (x - (sx * math.cos(a) - sy * math.sin(a)),
                 y - (sx * math.sin(a) + sy * math.cos(a)),
                 z - sz)
        distance = math.dist((0, 0, 0), to_go)

        if abs(turn) > self.yaw_tolerance:
            # Turn as far as keeps the branch in view; if that's hardly at all, move first
            bearing = math.degrees(math.atan2(y, x))
            allowed = max(bearing - self.keep_in_view, min(bearing + self.keep_in_view, turn))
            if abs(allowed) >= 1.0 or distance <= self.standoff_tolerance:
                return self.pacer.moved(YawBy(-allowed), now)  # yaw_by is clockwise positive

        # Approach: towards the standoff point, a step at a time
        if distance > self.standoff_tolerance:
            scale = min(1.0, self.step_length / distance)
            return self.pacer.moved(move(to_go, scale, self.approach_speed), now)

        # Final: one blind move that puts the branch on the capture point, then grasp
        self.stage = 'grasp'
        return self.pacer.moved(move([b - c for b, c in zip((x, y, z), self.capture)], 1.0,
                                     self.final_speed), now)


def move(offset, scale, speed):
    """MoveBody for an offset in base_link (y left), scaled down to a step."""
    forward, left, up = (c * scale for c in offset)
    return MoveBody(forward, -left, up, speed)
