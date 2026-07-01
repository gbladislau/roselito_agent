# roselito_agent

Executive functions for the ROSelito robot.

## Usage

To start the route recorder, call:
```bash
ros2 launch roselito_agent teach_route.launch save_path:=/absolute/path/to/route.pon
```

To start auto record path:
```bash
ros2 launch roselito_agent teach_route.launch auto_save:={bool} auto_save_frequency:={float}
```

The `save_path` argument can be omitted, in which case the route is saved to a file named `route.pon` in the current folder.

To add the robot's current pose to the route, call:
```bash
ros2 launch roselito_agent push_waypoint.launch
```

To erase the last pose added to the route, call:
```bash
ros2 launch roselito_agent pop_waypoint.launch
```

To save the route file, call:
```bash
ros2 launch roselito_agent save_route.launch
```

To save a route to a different path, instead of the path specified on the teach_route:
```bash
ros2 service call /teach_route/save roselito_interfaces/srv/SaveRouteInterface '{path: /absolute/path/to/route.pon }'
```

To replay a recorded route, call:
```bash
ros2 launch roselito_agent replay_route.launch path:=/absolute/path/to/route.pon
```

To set the squared path distance threshold, call:
```bash
ros2 launch roselito_agent replay_route.launch squared_threshold_distance:={float}
```

To configure the camera-based person proximity check, publish the detected person
position as a `geometry_msgs/msg/PointStamped` relative to the robot camera or
base frame and set the replay parameters:
```bash
ros2 launch roselito_agent replay_route.launch person_position_topic:=/person_position person_nearby_distance:=3.0 person_detection_timeout:=1.0
```

The `path` argument can be omitted, in which case the route is loaded from a file named `route.pon` in the current folder.

When replaying routes in simulation, make sure to add the `use_sim_time` argument:
```bash
ros2 launch roselito_agent replay_route.launch path:=/absolute/path/to/route.pon use_sim_time:=true
```
