# ackermann-odometry
ROS 2 Node (C++) to publish Odometry message for ackermann vehicle based on wheel speed and steering angle

## Usage

### Launching the Odometry Publisher

You can launch the odometry publisher node with default parameters from the config file:

```bash
ros2 launch ackermann_odometry odometry.launch.py
```

Or with a custom parameters file:

```bash
ros2 launch ackermann_odometry odometry.launch.py params_file:=/path/to/custom_params.yaml
```

### Parameters

Parameters are configured in `config/params.yaml`:

- `axle_length`: Distance between left and right wheels (meters), default: 1.0
- `wheelbase_length`: Distance between front and rear axles (meters), default: 2.0
- `wheel_radius`: Radius of the wheels (meters), default: 0.1
- `center_of_mass_offset`: Offset of center of mass from rear axle (meters), default: 0.0
- `damping_factor`: Damping factor for position updates, default: 1.0
- `publish_frequency`: Odometry publish frequency (Hz), default: 50.0
- `odom_topic`: Name of the odometry topic to publish to, default: "odom"
- `left_wheel_joint_name`: Name of the left wheel joint in JointState messages, default: "left_wheel_joint"
- `right_wheel_joint_name`: Name of the right wheel joint in JointState messages, default: "right_wheel_joint"
- `steering_joint_name`: Name of the steering joint in JointState messages, default: "steering_joint"

### Input Topics

The node subscribes to:
- `joint_states` (sensor_msgs/msg/JointState): Joint state messages containing wheel velocities and steering angle

### Configuration File

The default configuration file is located at `config/params.yaml`. You can:
1. Modify this file directly to set your vehicle's parameters
2. Create a custom configuration file and specify it using the `params_file` launch argument
3. Override parameters at runtime using ROS 2 parameter services:
   ```bash
   ros2 param set /odom_publisher axle_length 1.5
   ros2 param set /odom_publisher publish_frequency 100.0
   ros2 param set /odom_publisher odom_topic custom_odom
   ```

### Examples

**Change publish frequency:**
```bash
ros2 launch ackermann_odometry odometry.launch.py params_file:=/path/to/params.yaml
# In params.yaml, set publish_frequency: 100.0 for 100 Hz
```

**Change odometry topic name:**
```bash
ros2 launch ackermann_odometry odometry.launch.py params_file:=/path/to/params.yaml
# In params.yaml, set odom_topic: "custom_odom"
```
