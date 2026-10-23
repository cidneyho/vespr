"""Reading Gazebo's ground truth, shared by the vespr_sim nodes.

The nodes read gz-transport directly rather than through ros_gz_bridge, because the bridge
drops the entity names from pose messages. Gazebo gives a model's pose in the world and a
link's pose in its model, and doesn't say which model a link belongs to, so entities are
found by name and a name that appears more than once is refused.
"""
from vespr_sim.poses import compose


def find_pose(msg, name):
    """(pose, error) for the one entity called name in a gz Pose_V."""
    found = [p for p in msg.pose if p.name == name]
    if len(found) != 1:
        return None, f'{len(found)} entities called {name!r}' if found else f'no entity called {name!r}'
    p = found[0]
    position = (p.position.x, p.position.y, p.position.z)
    orientation = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    return (position, orientation), None


def link_in_world(msg, model, link):
    """(pose, error) of a model's link in the world: the model's world pose combined with the link's."""
    model_pose, error = find_pose(msg, model)
    if error:
        return None, error
    link_pose, error = find_pose(msg, link)
    if error:
        return None, error
    return compose(model_pose, link_pose), None


def stamp_ns(msg):
    """Sim time of a gz message, ns."""
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec


class RateLimit:
    """Lets samples through at on average at most `rate` a second, and never the same one twice.

    Works on sample times (sim time), so it follows the sim, not the wall clock.
    """

    def __init__(self, rate):
        self.period = 1e9 / rate
        self.slack = 0.05 * self.period  # so a source at exactly `rate` isn't thinned by jitter
        self.last = None  # ns, newest sample seen
        self.next_due = None  # ns, earliest sample time let through

    def due(self, stamp):
        if self.last is not None and stamp < self.last:
            self.next_due = None  # sim time went backwards: the sim was reset
        self.last = stamp
        if self.next_due is not None and stamp < self.next_due - self.slack:
            return False
        # Keep to the schedule, but after a gap restart it rather than catch up in a burst
        if self.next_due is None or stamp - self.next_due > self.period:
            self.next_due = stamp
        self.next_due += self.period
        return True
