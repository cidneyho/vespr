# ROS 2 primer, as used on VESPR

Board box 04. See the [index](README.md).

ROS 2 is what lets the programs inside the Jetson talk without knowing each other exist.

## The four ideas

- **Node: one program, one job.** Nodes never call each other's code; they only pass messages. John can rewrite the detector from scratch and the mission node does not change, as long as the messages look the same.
- **Topic: the bulletin board.** A named channel. A node publishes; any node subscribes. The publisher does not know who is listening. For streams: camera frames, IMU, position estimates, battery. Each topic carries one message type (the contract).
- **Service: the phone call.** A request with a reply; the asker waits. For quick things with a definite answer: zero the encoder, enable the motor drivers, reset the camera.
- **Action: the food order.** A long-running request with progress and cancel: goal, feedback, result. "Grasp the branch" is the textbook case: takes seconds, you want progress, you want a clear gripped / failed, and you want to be able to say stop.

| Kind | Use it for | Shape | On VESPR |
|---|---|---|---|
| TOPIC | Data that keeps coming | One way, many to many | Images, IMU, poses, leg state |
| SERVICE | A quick question or switch | Request, then one reply | Enable legs, reset camera |
| ACTION | A job that takes seconds | Goal, feedback, result, cancel | Grasp the branch, run a perch |
| PARAMETER | A setting on one node | Named value, read or set | Camera profile, thresholds |

## QoS: the silent trap

Every publisher and subscriber declares delivery: **reliable** (resend until it arrives; commands) or **best effort** (drop what is late; camera, IMU). A reliable subscriber on a best-effort publisher gets NOTHING, with no error.

PX4 publishes best effort, so **every subscriber to `/fmu/out/*` must ask for best effort.** Put it in the template node on day one. If a PX4 topic looks dead: `ros2 topic info -v`.

## Namespaces and remapping

Nodes use short names in code (`branch_pose`); the launch file decides where they live (`/vespr/perception/branch_pose`) and can remap any name with no code change.

This is the whole mocap phase-out: the mission node subscribes to "drone pose", and one launch-file line decides whether that is mocap or vision (see [mocap.md](mocap.md#phasing-out-mocap)).

## Frames: PX4 is NED, ROS is ENU

Every pose says relative to what. ROS: `map` (world), `odom`, `base_link` (body), `camera_link`; tf2 keeps the tree. PX4 uses North-East-Down with Z pointing down. **Convert in one node only.** The marker cluster is not `base_link`: measure the offset once or the comparison is biased. More in [naming.md](naming.md#frames).

## Debug commands

| Command | What it tells you |
|---|---|
| `ros2 node list` | which programs are running |
| `ros2 topic list -t` | every topic and its message type |
| `ros2 topic echo <topic>` | print messages as they arrive |
| `ros2 topic hz <topic>` | the rate it is really publishing at |
| `ros2 topic info -v <topic>` | who publishes, who subscribes, which QoS |
| `ros2 service call <name> <type> <req>` | try a service by hand |
| `ros2 action send_goal <name> <type> <goal>` | try an action by hand |
| `rqt_graph` | draw the whole graph |
| `ros2 bag record -a` | record every topic to replay later |
