# CARLA Collision-Avoidance Scenario

A ROS 2 package for a CARLA vehicle-collision avoidance scenario.

`carla_agent` monitors forward radar detections and the relative approach
velocity of obstacles. It progressively brakes inside a safety distance and
applies an emergency brake when an obstacle reaches the configured braking
distance. The package also includes vehicle spawning, object detection, lane
following, visualization, and a launch configuration.

## Package layout

- `src/carla_agent/carla_agent/autopilot_node.py` — radar-based collision avoidance and emergency braking
- `src/carla_agent/carla_agent/vehicle_spawner.py` — CARLA vehicle spawning
- `src/carla_agent/carla_agent/object_detector.py` — object-detection support
- `src/carla_agent/carla_agent/lane_detect.py` and `lane_follower.py` — lane perception and following
- `src/carla_agent/launch/carla_agent.launch.py` — ROS 2 launch file

## Build

```bash
source /opt/ros/galactic/setup.bash
colcon build --packages-select carla_agent
source install/setup.bash
```

Build artifacts are intentionally excluded from version control.
