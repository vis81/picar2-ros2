"""Scenario -> Gazebo SDF world.

Physics is pinned deliberately. Gazebo Harmonic exposes no RNG seed, so
bit-exact repeats are impossible; what *is* controllable is the real-time
factor. Left unpinned, gz targets RTF 1.0 and silently degrades under load,
which changes how much wall-clock compute the ROS stack gets per simulated
second and therefore changes behaviour. Pinning it below 1.0 removes that as a
variable, and the runner rejects trials whose achieved RTF drifts.
"""
from __future__ import annotations

from .spec import WORLD_NAME, Box, Scenario

_PLUGINS = """
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
"""

_LIGHT_AND_GROUND = """
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'><geometry><plane><normal>0 0 1</normal>
          <size>200 200</size></plane></geometry></collision>
        <visual name='visual'><geometry><plane><normal>0 0 1</normal>
          <size>200 200</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
"""

WALL_HEIGHT = 1.0


def _box_model(name: str, b: Box) -> str:
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{b.x} {b.y} {WALL_HEIGHT / 2} 0 0 0</pose>
      <link name='link'>
        <collision name='c'><geometry><box><size>{b.sx} {b.sy} {WALL_HEIGHT}</size>
          </box></geometry></collision>
        <visual name='v'><geometry><box><size>{b.sx} {b.sy} {WALL_HEIGHT}</size>
          </box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient>
            <diffuse>0.6 0.6 0.65 1</diffuse></material></visual>
      </link>
    </model>"""


def to_sdf(sc: Scenario) -> str:
    bodies = [_box_model(f'wall_{i}', b) for i, b in enumerate(sc.walls)]
    bodies += [_box_model(f'obstacle_{i}', b) for i, b in enumerate(sc.obstacles)]
    return f"""<sdf version='1.6'>
  <world name='{WORLD_NAME}'>
    <physics name='pinned' type='ode'>
      <max_step_size>{sc.max_step}</max_step_size>
      <real_time_factor>{sc.rtf}</real_time_factor>
    </physics>
{_PLUGINS}{_LIGHT_AND_GROUND}{''.join(bodies)}
  </world>
</sdf>
"""
