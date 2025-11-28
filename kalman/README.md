# kalman meta-package

## Command cheat sheets

`makerspet_mini` is the default robot model. When using another robot model, change the default model using Kaia.ai CLI.
For example, the command below sets the default robot model to `makerspet_loki`.
```
kaia config robot.model makerspet_loki
```

## Operate a physical robot

### Manual operation

```
# Launch the physical robot
ros2 launch kaiaai_bringup physical.launch.py

# Drive robot manually
ros2 run kaiaai_teleop teleop_keyboard

# Monitor robot sensors
ros2 launch kaiaai_bringup monitor_robot.launch.py

# Create a map while driving manually
ros2 launch kaiaai_bringup cartographer.launch.py

# Save the newly-created map
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0
```

### Physical robot self-drives automatically
```
# Launch the physical robot
ros2 launch kaiaai_bringup physical.launch.py

# Specify target location;; robot self-drives using an existing map
ros2 launch kaiaai_bringup navigation.launch.py map:=$HOME/maps/map.yaml

# Launch SLAM (simultaneous localization and mapping) - navigate and map simultaneously
ros2 launch kaiaai_bringup navigation.launch.py slam:=True

# Robot automatically seeks out, self-drives to unknown locations
ros2 launch explore_lite explore.launch.py

# Save the newly-created map
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0
```

## Use Command Line with Physical Robot

### Get, Set Robot's Parameters
```
# View parameters
ros2 node list
ros2 node info /pet
ros2 param list /pet
ros2 param dump /pet

# Get the current laser scan frequency
ros2 param get /pet lidar.scan.freq.now

# Set the desired laser scan frequency to 7 Hz
ros2 param set /pet lidar.scan.freq.target 7.0

# Get the current desired laser scan frequency
ros2 param get /pet lidar.scan.freq.target

# Reset the desired laser scan frequency to default
ros2 param set /pet lidar.scan.freq.target 0.0
```

### Monitor Robot's Telemetry
```
# List available topics
ros2 topic list

# Get WiFi strength
ros2 topic echo /wifi_state --once

# View raw telemetry
ros2 topic echo /telemetry --once

# Get LiDAR scan data
ros2 topic echo /scan --once

# View current odometer value
ros2 topic echo /odom --once

# View current wheel rotation angles
ros2 topic echo /joint_states --once

# View current battery voltage, charge percentage
ros2 topic echo /battery_state --once

# View target velocity sent by kaiaai_telem or navigation
ros2 topic echo /cmd_vel --once
```

## Operate robot in simulated environment

### Operate a simulated robot manually

```
# Launch the robot in a simulation - drive manually
ros2 launch kaiaai_gazebo world.launch.py
ros2 run kaiaai_teleop teleop_keyboard
ros2 launch kaiaai_bringup monitor_robot.launch.py

# Launch the robot in a simulation - robot self-drives around
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py
ros2 launch kaiaai_bringup monitor_robot.launch.py

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true \
  map:=/ros_ws/src/kaiaai_gazebo/map/living_room.yaml

# Launch the robot in a simulation - navigate and create a map simultaneously; save the map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0
```

### Let robot self-drive autonomously

```
# Launch the robot in a simulation - create, save a map; robot self-drives around trivially
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/living_room_map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 launch explore_lite explore.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping, saves map
ros2 run auto_mapper auto_mapper map_path:=/ros_ws/map.yaml
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 run nav2_wfd explore
```
