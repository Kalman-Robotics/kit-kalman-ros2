## On microcontroller
Terminal
```
Telem avg 51 max 52ms, LiDAR RPM 10.00, wheels RPM 0.00 0.00, battery 3.70V, RSSI -29dBm
```

## On PC

Incoming Topics
```
/buzzer [kalman_interfaces/msg/Buzzer]
/cmd_vel [geometry_msgs/msg/Twist]
/imu_telem [kalman_interfaces/msg/ImuData]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/telemetry [kalman_interfaces/msg/KaiaaiTelemetry2]
```

TELEMETRY topic content
```
---
stamp:
  sec: 1762967915
  nanosec: 874411000
seq: 193171
odom_pos_x: 0.026938902214169502
odom_pos_y: 0.5149940848350525
odom_pos_yaw: -0.1071779727935791
odom_vel_x: 0.0
odom_vel_yaw: 0.0
joint:
- pos: -124.45195007324219
  vel: 0.0
- pos: 123.92835235595703
  vel: 0.0
wifi_rssi_dbm: -33
battery_mv: 3700
distance_mm: []
bumper: []
cliff: []
touch: []
scan_start_hint: false
lds: [...]
---
```

### ROS2 nodes
Telemetry node
```
/battery_state
/buzzer
/cmd_vel
/control_status
/imu_telem
/joint_states
/odom
/parameter_events
/rosout
/scan
/telemetry
/tf
/wifi_state
```
