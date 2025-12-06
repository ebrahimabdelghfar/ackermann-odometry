# ackermann-odometry
ROS Node to publish Odometry message for ackermann vehicle based on wheel speed and steering angle

## Usage

### Launching the Odometry Publisher

You can launch the odometry publisher node with default parameters:

```bash
ros2 launch ackermann_odometry odometry.launch.py
```

Or with custom parameters:

```bash
ros2 launch ackermann_odometry odometry.launch.py axle_length:=1.5 wheelbase_length:=2.5 wheel_radius:=0.15
```

### Parameters

- `axle_length`: Distance between left and right wheels (meters), default: 1.0
- `wheelbase_length`: Distance between front and rear axles (meters), default: 2.0
- `wheel_radius`: Radius of the wheels (meters), default: 0.1
- `center_of_mass_offset`: Offset of center of mass from rear axle (meters), default: 0.0
- `damping_factor`: Damping factor for position updates, default: 1.0
