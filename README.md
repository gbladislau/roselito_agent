# roselito_agent

Executive functions for the ROSelito robot.

## Nodes

| Node | Description |
|---|---|
| `teach_route` | Records the robot's pose as waypoints into a route file |
| `replay_route` | Replays a saved route, publishing waypoints to the Nav2 stack |
| `person_distance` | Detects a person via RealSense + MobileNet-SSD and publishes their distance |

---

## Teaching a route

Start the route recorder:
```bash
ros2 launch roselito_agent teach_route.launch save_path:=/absolute/path/to/route.pon
```

To enable automatic periodic recording:
```bash
ros2 launch roselito_agent teach_route.launch auto_save:=true auto_save_frequency:=1.0
```

The `save_path` argument can be omitted, in which case the route is saved to `route.pon` in the current folder.

Add the robot's current pose to the route:
```bash
ros2 launch roselito_agent push_waypoint.launch
```

Erase the last pose added:
```bash
ros2 launch roselito_agent pop_waypoint.launch
```

Save the route file:
```bash
ros2 launch roselito_agent save_route.launch
```

Save to a different path than the one specified at startup:
```bash
ros2 service call /teach_route/save roselito_interfaces/srv/SaveRouteInterface '{path: /absolute/path/to/route.pon}'
```

---

## Replaying a route

```bash
ros2 launch roselito_agent replay_route.launch path:=/absolute/path/to/route.pon
```

The `path` argument can be omitted, in which case the route is loaded from `route.pon` in the current folder.

For simulation, add `use_sim_time`:
```bash
ros2 launch roselito_agent replay_route.launch path:=/absolute/path/to/route.pon use_sim_time:=true
```

### replay_route parameters

| Parameter | Default | Description |
|---|---|---|
| `path` | `$(PWD)/route.pon` | Route file to replay |
| `frequency` | `20.0` Hz | Timer callback rate |
| `squared_threshold_distance` | `1.0` | Squared distance (m²) to consider a waypoint reached |
| `global_frame` | `map` | TF global reference frame |
| `robot_base_frame` | `base_footprint` | TF robot base frame |
| `person_position_topic` | `/person_position` | Topic carrying person detections |
| `person_nearby_distance` | `1.5` m | Maximum distance to count the person as "nearby" |
| `person_detection_timeout` | `1.0` s | How stale a detection can be before it is ignored |
| `person_miss_limit` | `40` ticks | Consecutive ticks without a nearby person before the robot pauses (2 s at 20 Hz) |

Set the waypoint distance threshold:
```bash
ros2 launch roselito_agent replay_route.launch squared_threshold_distance:=0.5
```

---

## Person-proximity following (RealSense camera)

The `replay_route` launch file automatically starts the `person_distance` node alongside the route replayer.
The robot navigates waypoints only while a person is detected within `person_nearby_distance` metres.
If the person moves out of range or disappears from the camera, the robot holds its current goal until
the person returns.

A short **miss buffer** (`person_miss_limit` ticks) prevents the robot from stopping on momentary
detection glitches. At the default 20 Hz rate, `person_miss_limit:=20` gives a 1-second grace window
before the robot pauses.

### How it works

```
RealSense camera
      |  colour + depth frames (depth aligned to colour)
      v
[person_distance node]
  MobileNet-SSD detects person in colour frame
  Samples median depth in a patch around the bounding-box centre
  Publishes geometry_msgs/PointStamped -> /person_position
      |
      v
[replay_route node]
  Every tick:
    1. Publishes current goal -> /goal_pose  (always, so Nav2 stays alive)
    2. If person NOT seen recently OR distance > person_nearby_distance:
         increment miss counter
         if miss counter >= person_miss_limit: hold, do not advance waypoint
    3. If person IS nearby:
         reset miss counter
         check TF distance to goal -> advance waypoint index when close enough
```

### person_distance parameters

| Parameter | Default | Description |
|---|---|---|
| `model_config` | `$(PWD)/models/MobileNetSSD_deploy.prototxt` | Path to MobileNet-SSD prototxt |
| `model_weights` | `$(PWD)/models/MobileNetSSD_deploy.caffemodel` | Path to MobileNet-SSD caffemodel |
| `detection_frequency` | `10.0` Hz | How often to process camera frames |
| `confidence_threshold` | `0.4` | Minimum DNN detection confidence |
| `image_scale` | `1.5` | Upscale factor applied before detection (helps range) |
| `frame_width` | `640` | RealSense colour/depth stream width |
| `frame_height` | `480` | RealSense colour/depth stream height |
| `camera_frame` | `camera_color_optical_frame` | TF frame id stamped on published messages |

### Detection back-ends

The node selects the detector at startup based on whether model files are provided:

| Back-end | Effective range | Angle robustness | Trigger |
|---|---|---|---|
| MobileNet-SSD (DNN) | ~0.5 m to 8+ m | Robust | `model_config` and `model_weights` both set |
| HOG (fallback) | ~0.5 m to 2 m | Frontal only | Either model path left empty |

### Model files

The model files are stored in `models/` inside the package:

```
roselito_agent/models/
  MobileNetSSD_deploy.prototxt    (~29 KB)
  MobileNetSSD_deploy.caffemodel  (~23 MB)
```

To re-download them if lost:
```bash
cd /path/to/roselito_agent
mkdir -p models

wget "https://raw.githubusercontent.com/djmv/MobilNet_SSD_opencv/master/MobileNetSSD_deploy.prototxt" \
     -O models/MobileNetSSD_deploy.prototxt

wget "https://github.com/djmv/MobilNet_SSD_opencv/raw/master/MobileNetSSD_deploy.caffemodel" \
     -O models/MobileNetSSD_deploy.caffemodel
```

### Tuning tips

- **Person not detected at distance** — lower `confidence_threshold` (try `0.3`) or raise `image_scale` (try `2.0`).
- **Too many false stops** — raise `person_miss_limit` (e.g. `40` = 2 s at 20 Hz).
- **Robot pauses too slowly** — lower `person_miss_limit` (e.g. `10` = 0.5 s).
- **Range still insufficient** — the DNN detector works best with a standing, mostly-visible person. Obstructions, poor lighting, or the person facing away will reduce range.

### Running person_distance in isolation (for debugging)

```bash
cd /path/to/roselito_agent
ros2 run roselito_agent person_distance \
  --ros-args \
  -p model_config:=$(pwd)/models/MobileNetSSD_deploy.prototxt \
  -p model_weights:=$(pwd)/models/MobileNetSSD_deploy.caffemodel

# Watch detections in another terminal
ros2 topic echo /person_position
```

### Simulating a person detection manually

```bash
# Simulate person at 2 m (robot should go)
ros2 topic pub --once /person_position geometry_msgs/msg/PointStamped \
  '{header: {stamp: {sec: 0}, frame_id: "camera_color_optical_frame"}, point: {x: 2.0, y: 0.0, z: 0.0}}'

# Stop publishing and wait > person_detection_timeout to confirm robot pauses
```
