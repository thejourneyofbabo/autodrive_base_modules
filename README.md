# Autodrive Base Modules

Base ROS2 packages for Autodrive Simulator integration.

## autodrive_ackermann

Ackermann drive interface for Autodrive Simulator with odometry publishing.

### Features
- Converts Ackermann drive commands to Autodrive throttle/steering
- Publishes odometry from simulator sensors (speed, IMU, encoders, steering)
- Automatic vehicle geometry detection via TF lookup

### Quick Start

```bash
# Launch both nodes
ros2 launch autodrive_ackermann autodrive_ackermann.launch.py vehicle_name:=roboracer_1

# Or launch individually
ros2 run autodrive_ackermann ackermann_to_autodrive_node
ros2 run autodrive_ackermann autodrive_to_odom_node
```

### Topics

**Subscribed:**
- `/drive` - AckermannDriveStamped commands
- `/autodrive/{vehicle_name}/speed` - Vehicle speed (Float32)
- `/autodrive/{vehicle_name}/imu` - IMU data
- `/autodrive/{vehicle_name}/steering` - Steering angle (Float32)
- `/autodrive/{vehicle_name}/left_encoder` - Left wheel encoder (JointState)
- `/autodrive/{vehicle_name}/right_encoder` - Right wheel encoder (JointState)

**Published:**
- `/autodrive/{vehicle_name}/throttle_command` - Throttle control (Float32)
- `/autodrive/{vehicle_name}/steering_command` - Steering control (Float32)
- `/odom` - Odometry (nav_msgs/Odometry)
- TF: `odom` → `base_link`

### Configuration

Edit [config/autodrive_params.yaml](autodrive_ackermann/config/autodrive_params.yaml) to tune parameters.
