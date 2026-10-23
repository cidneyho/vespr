#!/usr/bin/env python3
"""Generate the x500_hook model: the PX4 X500 with our hook on top.

The hook geometry comes from perch_flight's config/hook.yaml, so the sim hook
always matches what perch_approach flies with. Running this script writes
  models/x500_hook/model.sdf
  models/x500_hook/model.config

Usage: python3 make_hook_model.py   (then restart the sim)

The hook, in the drone's body frame (x forward, y left, z up, origin at the
body origin PX4 reports as its position):
  post   straight up from the top of the body to the bottom of the ring
  ring   centred `height` above the body origin, inside diameter `hole_diameter`,
         in the vertical plane that contains the direction the opening faces
  fixed  the back half of the ring, plus a little over the top so the branch
         rests on the fixed part when hanging
  gate   the front half. Open, it sits beside the fixed half. Closing swings it
         180 deg round the ring's axis, so it slides along the ring and never
         cuts through the inside where the branch is.

The gate is driven by a JointPositionController on Gazebo topic /hook/gate (angle,
0 open, pi closed). Two TriggeredPublishers turn a gz.msgs.Boolean on /hook/close
into that angle, so ROS 2 only has to publish std_msgs/Bool on /hook/close through
ros_gz_bridge.

The camera comes from the drone section's camera block: a Gazebo RGB-D camera with the
RealSense D435i's colour view, on link camera_link fixed to base_link. Gazebo publishes it
on /camera/image, /camera/depth_image, /camera/points and /camera/camera_info.

The flow and rangefinder blocks add a downward optical flow sensor and a downward single-beam
lidar, built as on PX4's x500_flow. PX4's simulation bridge finds them by name (flow_link,
and lidar_sensor_link with sensor lidar), so keep those names. run_sim.sh starts this model
with PX4's flow airframe, which flies on them with GPS off.
"""
import math
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
MODEL_DIR = HERE.parent / 'models' / 'x500_hook'
HOOK_YAML = HERE.parent.parent / 'perch_flight' / 'config' / 'hook.yaml'

BODY_TOP = 0.03       # m above the body origin where the post starts (top plate)
FIXED_OVER_TOP = 15   # deg the fixed half reaches past the top into the front
SEGMENTS = 12         # straight cylinders per half ring
FIXED_MASS = 0.03     # kg
GATE_MASS = 0.01      # kg
CAMERA_MASS = 0.072   # kg, D435i
CAMERA_SIZE = (0.025, 0.090, 0.025)  # m, D435i case: depth, width, height
RANGEFINDER_MASS = 0.001  # kg, as PX4's x500_flow


def load_params():
    params = yaml.safe_load(HOOK_YAML.read_text())['perch_approach']['ros__parameters']
    return params['hook'], params['drone']


def pose_along(p0, p1):
    """Pose of a cylinder from p0 to p1: midpoint, and rpy turning z onto the segment."""
    d = [b - a for a, b in zip(p0, p1)]
    length = math.sqrt(sum(c * c for c in d))
    mid = [(a + b) / 2 for a, b in zip(p0, p1)]
    pitch = math.acos(max(-1.0, min(1.0, d[2] / length)))
    yaw = math.atan2(d[1], d[0])
    return mid, (0.0, pitch, yaw), length


def cylinder(name, p0, p1, radius, extra=0.0):
    mid, rpy, length = pose_along(p0, p1)
    pose = ' '.join(f'{v:.5f}' for v in (*mid, *rpy))
    geometry = (f'<geometry><cylinder><radius>{radius:.4f}</radius>'
                f'<length>{length + extra:.4f}</length></cylinder></geometry>')
    return (f'      <collision name="{name}">\n'
            f'        <pose>{pose}</pose>\n        {geometry}\n'
            f'      </collision>\n'
            f'      <visual name="{name}_visual">\n'
            f'        <pose>{pose}</pose>\n        {geometry}\n'
            f'        <material><ambient>{{color}}</ambient><diffuse>{{color}}</diffuse></material>\n'
            f'      </visual>\n')


def arc(prefix, centre, radius, forward, from_deg, to_deg, wire_r, side=(0.0, 0.0, 0.0)):
    """Cylinders along the ring from from_deg to to_deg.

    Angles are in the ring's plane: 0 = the way the opening faces, 90 = top,
    180 = back, 270 = bottom.
    """
    def point(deg):
        a = math.radians(deg)
        return [c + s + radius * (math.cos(a) * f + math.sin(a) * z)
                for c, s, f, z in zip(centre, side, forward, (0.0, 0.0, 1.0))]
    out = ''
    for i in range(SEGMENTS):
        a0 = from_deg + (to_deg - from_deg) * i / SEGMENTS
        a1 = from_deg + (to_deg - from_deg) * (i + 1) / SEGMENTS
        # A wire diameter longer than the chord, so neighbouring segments overlap at the joints
        out += cylinder(f'{prefix}_{i}', point(a0), point(a1), wire_r, extra=2 * wire_r)
    return out


def inertial(mass):
    i = mass * 0.05 ** 2
    return (f'      <inertial>\n        <mass>{mass}</mass>\n'
            f'        <inertia><ixx>{i:.2e}</ixx><iyy>{i:.2e}</iyy><izz>{i:.2e}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>\n      </inertial>\n')


def camera_xml(camera):
    """RGB-D camera link with the D435i's colour view, fixed to base_link."""
    pose = ' '.join(f'{float(v):.5f}' for v in (
        camera['x'], camera['y'], camera['z'], 0.0, -math.radians(float(camera['tilt_up_deg'])), 0.0))
    size = ' '.join(f'{v:.3f}' for v in CAMERA_SIZE)
    near, far = float(camera['near']), float(camera['far'])
    return f'''    <!-- RealSense D435i stand-in. Images on /camera/image, /camera/depth_image, /camera/camera_info -->
    <link name="camera_link">
      <pose relative_to="base_link">{pose}</pose>
{inertial(CAMERA_MASS)}      <visual name="camera_visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor name="realsense_d435i" type="rgbd_camera">
        <always_on>1</always_on>
        <update_rate>{float(camera['rate'])}</update_rate>
        <topic>camera</topic>
        <camera>
          <horizontal_fov>{math.radians(float(camera['horizontal_fov_deg'])):.5f}</horizontal_fov>
          <image><width>{int(camera['width'])}</width><height>{int(camera['height'])}</height><format>R8G8B8</format></image>
          <clip><near>{near}</near><far>{far}</far></clip>
          <depth_camera><clip><near>{near}</near><far>{far}</far></clip></depth_camera>
        </camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
'''


def flow_xml(flow):
    """PX4's optical_flow model (downward camera + flow sensor, link flow_link), fixed to base_link."""
    xyz = ' '.join(f'{float(flow[k]):.5f}' for k in ('x', 'y', 'z'))
    return f'''    <!-- Optical flow, looking down. PX4 reads /link/flow_link/sensor/optical_flow/optical_flow -->
    <include merge="true">
      <uri>model://optical_flow</uri>
      <pose relative_to="base_link">{xyz} 0 0 0</pose>
    </include>
    <joint name="optical_flow_joint" type="fixed">
      <parent>base_link</parent>
      <child>flow_link</child>
    </joint>
'''


def rangefinder_xml(rangefinder):
    """Single-beam lidar looking down on lidar_sensor_link, as PX4's x500_flow."""
    xyz = ' '.join(f'{float(rangefinder[k]):.5f}' for k in ('x', 'y', 'z'))
    near, far = float(rangefinder['min_range']), float(rangefinder['max_range'])
    return f'''    <!-- Rangefinder, looking down. PX4 reads /link/lidar_sensor_link/sensor/lidar/scan -->
    <link name="lidar_sensor_link">
      <!-- Pitched 90 deg so the beam points down -->
      <pose relative_to="base_link">{xyz} 0 1.5708 0</pose>
{inertial(RANGEFINDER_MASS)}      <visual name="rangefinder_visual">
        <geometry><box><size>0.02 0.03 0.02</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0 3.14 0 0</pose>
        <update_rate>50</update_rate>
        <ray>
          <scan>
            <horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range><min>{near}</min><max>{far}</max><resolution>0.01</resolution></range>
        </ray>
        <always_on>1</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>
    <joint name="lidar_sensor_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_sensor_link</child>
    </joint>
'''


def build(hook, drone):
    camera = drone['camera']
    hole_d, height = float(hook['hole_diameter']), float(hook['height'])
    facing, wire_d = float(hook['facing_deg']), float(hook['wire_diameter'])
    close_time = float(hook['close_time'])
    r = (hole_d + wire_d) / 2          # ring centre line, so the inside diameter is hole_d
    wire_r = wire_d / 2
    if height - r - wire_r <= BODY_TOP:
        raise SystemExit(f'hook height {height} m is too small: the ring would sit in the body')

    # facing_deg is clockwise from the nose seen from above; body y points left
    f = math.radians(facing)
    forward = (math.cos(f), -math.sin(f), 0.0)
    axis = (math.sin(f), math.cos(f), 0.0)       # ring axis, horizontal, across the opening
    centre = (0.0, 0.0, height)
    origin = (0.0, 0.0, 0.0)
    # Gate sits one wire diameter to the side of the fixed half, so they don't overlap
    side = tuple(wire_d * a for a in axis)

    fixed = cylinder('post', (0.0, 0.0, BODY_TOP), (0.0, 0.0, height - r), wire_r)
    fixed += arc('ring', centre, r, forward, 90 - FIXED_OVER_TOP, 270, wire_r)
    # Gate geometry is drawn open (over the back half), in a link whose origin is the ring centre
    gate = arc('gate', origin, r, forward, 90, 270, wire_r, side)

    a = ' '.join(f'{v:.5f}' for v in axis)
    c = ' '.join(f'{v:.5f}' for v in centre)
    return f'''<?xml version="1.0"?>
<!--
  Generated by perch_sim/scripts/make_hook_model.py from perch_flight/config/hook.yaml.
  Edit hook.yaml and rerun the script. Don't edit this file by hand.

  PX4 X500 with a hook on a post from the centre of the body.
  Ring: {hole_d * 100:.1f} cm inside diameter, {wire_d * 1000:.0f} mm wire, centre {height * 100:.1f} cm
    above the body origin, opening facing {facing:.0f} deg clockwise from the nose.
  Gate: swings 180 deg round the ring to close it in about {close_time:.1f} s.
    Gazebo topic /hook/close (gz.msgs.Boolean): true closes, false opens.
  Camera: RealSense D435i stand-in on camera_link, {float(camera['x']) * 100:.1f} cm forward,
    {float(camera['z']) * 100:.1f} cm up, tilted {float(camera['tilt_up_deg']):.0f} deg up.
    {float(camera['horizontal_fov_deg']):.0f} deg wide, {int(camera['width'])} x {int(camera['height'])} at {float(camera['rate']):.0f} Hz.
  Optical flow and a rangefinder, both looking down (mounts are placeholders from PX4's x500_flow).
-->
<sdf version="1.9">
  <model name="x500_hook">
    <include merge="true">
      <uri>model://x500</uri>
    </include>

    <!-- base_link is 0.24 m above the model origin, so place the hook relative to it -->
    <link name="hook">
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
{inertial(FIXED_MASS)}{fixed.replace('{color}', '0.15 0.15 0.15 1')}    </link>
    <joint name="hook_joint" type="fixed">
      <parent>base_link</parent>
      <child>hook</child>
    </joint>

    <link name="hook_gate">
      <pose relative_to="base_link">{c} 0 0 0</pose>
{inertial(GATE_MASS)}{gate.replace('{color}', '0.85 0.45 0.05 1')}    </link>
    <joint name="hook_gate_joint" type="revolute">
      <parent>hook</parent>
      <child>hook_gate</child>
      <axis>
        <xyz expressed_in="__model__">{a}</xyz>
        <limit><lower>-0.1</lower><upper>3.3</upper><effort>20</effort></limit>
      </axis>
    </joint>

    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>hook_gate_joint</joint_name>
      <topic>/hook/gate</topic>
      <use_velocity_commands>true</use_velocity_commands>
      <cmd_max>{math.pi / close_time:.3f}</cmd_max>
      <cmd_min>{-math.pi / close_time:.3f}</cmd_min>
    </plugin>
    <plugin filename="gz-sim-triggered-publisher-system"
            name="gz::sim::systems::TriggeredPublisher">
      <input type="gz.msgs.Boolean" topic="/hook/close"><match>data: true</match></input>
      <output type="gz.msgs.Double" topic="/hook/gate">data: {math.pi:.5f}</output>
    </plugin>
    <plugin filename="gz-sim-triggered-publisher-system"
            name="gz::sim::systems::TriggeredPublisher">
      <input type="gz.msgs.Boolean" topic="/hook/close"><match>data: false</match></input>
      <output type="gz.msgs.Double" topic="/hook/gate">data: 0</output>
    </plugin>
    <!-- Gate angle on /hook/joint_state: 0 open, pi closed -->
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
      <topic>/hook/joint_state</topic>
      <joint_name>hook_gate_joint</joint_name>
    </plugin>

{camera_xml(camera)}
{flow_xml(drone['flow'])}
{rangefinder_xml(drone['rangefinder'])}  </model>
</sdf>
'''


def main():
    hook, drone = load_params()
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    (MODEL_DIR / 'model.sdf').write_text(build(hook, drone))
    (MODEL_DIR / 'model.config').write_text('''<?xml version="1.0"?>
<model>
  <name>x500_hook</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>PX4 X500 with the perching hook on top, a RealSense D435i stand-in, and downward optical flow and rangefinder. Generated by scripts/make_hook_model.py.</description>
</model>
''')
    print(f'wrote {MODEL_DIR / "model.sdf"}')


if __name__ == '__main__':
    main()
