# Design: legs interface and sim bridge

Status: draft for review. Nothing is implemented yet.

## Goal

Give the mission one interface for the legs that works the same whether it talks to the real legs or to the Gazebo hook. This doc covers the interface and the sim side. The real ESP32 bridge comes later.

## Interface

All four are served by a node named `legs_bridge`.

| Kind | Name | Type | Meaning |
|---|---|---|---|
| Topic | `/vespr/legs/state` | `sensor_msgs/msg/JointState` | Leg position, 10 Hz |
| Service | `/vespr/legs/set_enabled` | `std_srvs/srv/SetBool` | Motors on or off. Also the software stop |
| Action | `/vespr/legs/grasp` | `vespr_interfaces/action/Grasp` | Close until gripped |
| Action | `/vespr/legs/release` | `vespr_interfaces/action/Grasp` | Open |

**State message.** One joint, `name = ["latch"]`. `position` is 0 (open) to 1 (closed). `effort` is motor current in A, left empty when unknown (as in sim). "Gripped" is not part of the state; it comes from the grasp result.

**`Grasp.action`**
```
# goal: none
---
bool success
bool gripped
string message
---
float32 latch_position   # 0 open .. 1 closed
bool moving
```

**Rules for both bridges**
- Starts disabled. Grasp and release are rejected while disabled.
- Disabling stops the motors and aborts any running goal.
- One goal at a time. A new grasp or release goal pre-empts the current one.
- Cancelling a goal stops the motors where they are.
- A goal aborts if it hasn't finished within a timeout (parameter, default 3 s).

## Packages

| Package | Build | Contents |
|---|---|---|
| `vespr_interfaces` | ament_cmake | `action/Grasp.action` |
| `vespr_legs` | ament_python | `legs_bridge_sim` node, `sim.launch.py`, unit tests |

The real bridge goes in `vespr_legs` later. The sim executable is `legs_bridge_sim`, but its node name is `legs_bridge`, so the sim's ROS graph matches the real one.

## Sim node

The hook model already has what we need:

| Gazebo topic | Type | Use |
|---|---|---|
| `/hook/gate` | `gz.msgs.Double` | Gate target angle: 0 open, π closed. Closes in about 1 s |
| `/hook/joint_state` | `gz.msgs.Model` | Measured gate angle |

`sim.launch.py` starts `ros_gz_bridge` for those two topics (`std_msgs/Float64` to Gazebo, `sensor_msgs/JointState` from Gazebo), plus the node, with `use_sim_time` on.

| Interface | What the sim node does |
|---|---|
| state | Publishes the gate angle / π as the latch position |
| grasp | Sends π to `/hook/gate`. Succeeds with `gripped = true` once the angle is within 0.05 rad of π |
| release | Sends 0. Succeeds once the angle is within 0.05 rad of 0 |
| cancel, disable, timeout | Sends the current angle, so the gate stops where it is |

**Decision for review:** the node commands `/hook/gate` directly, not `/hook/close`. `/hook/close` can only say "fully open" or "fully closed", so cancel and the software stop couldn't stop the gate mid-swing. `/hook/close` keeps working for manual tests.

**Sim limit:** "gripped" just means the gate closed. The sim doesn't check whether the rod is inside the hook.

## Testing

- Unit tests for the pure logic: angle to latch position, the done check, and the enabled/goal rules.
- Manual: `run_sim.sh perch_rod gz_x500_hook`, then `ros2 launch vespr_legs sim.launch.py`. Then call `set_enabled`, send grasp and release goals with `ros2 action send_goal`, and cancel one mid-swing.

## Out of scope

- The real `legs_bridge` (ESP32 link undecided).
- Mission-side use of the interface.
- A real "gripped" sensor.
