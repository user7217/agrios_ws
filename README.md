# Agrios Robot Workspace: Comprehensive Documentation

Welcome to the **Agrios Robot Workspace** (`agrios_ws`). This is a ROS 2 workspace dedicated to the simulation, control, and state estimation of the **Agrios** autonomous agricultural and outdoor platform.

This document serves as an exhaustive guide to the architecture, the specific mechanics of each package, and the underlying math and code that makes the simulation and localization stack function.

---

## 🏗️ 1. Architecture & File Explanations

The workspace is structured following standard ROS 2 conventions, divided into three main packages.

### 1.1. `agrios_description` (Simulation & Hardware Definition)
This package is entirely responsible for defining what the robot looks like, how it behaves physically, and how it spawns into the Gazebo (Ignition) simulation.

#### `urdf/agrios.urdf.xacro`
This is the core robot description file. It uses XACRO (XML Macros) to programmatically generate a URDF.

**Key Feature: Lockable Steering**
The file accepts an argument `lock_steering`. This allows the robot to seamlessly switch between being a skid-steer vehicle (like a tank) or an Ackermann-steered vehicle (like a car).

```xml
  <!-- ===== ARGS ===== -->
  <xacro:arg name="lock_steering" default="true"/>
  <xacro:property name="lock_steer" value="$(arg lock_steering)"/>
  <xacro:property name="steer_joint_type"
                  value="${'fixed' if lock_steer == 'true' else 'revolute'}"/>
```
If `true`, the `steer_joint_type` becomes `fixed`, locking the wheels straight forward. If `false`, it becomes `revolute`, allowing a joint controller to steer them.

**Gazebo Plugins**
The `.xacro` file doesn't just define shapes; it defines how the simulation engine interacts with the robot via plugins. For instance, the DiffDrive plugin is used to move the robot:

```xml
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system"
            name="ignition::gazebo::systems::DiffDrive">
      <left_joint>fl_wheel_joint</left_joint>
      <left_joint>rl_wheel_joint</left_joint>
      <!-- ... -->
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
    </plugin>
```

#### `launch/sim.launch.py`
This is the main entry point to start the physics engine. It performs several crucial steps:
1.  **Starts Gazebo** with the `agrios_empty.sdf` world.
2.  **Spawns the Robot** dynamically using a `TimerAction` to ensure Gazebo is ready.
3.  **Establishes the Bridge**. Gazebo uses Ignition Transport, but ROS 2 uses DDS. The `ros_gz_bridge` is essential to translate data between them.

```python
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            # ...
        ])
```
Notice the syntax: `[topic]@[ros_type][direction][ign_type]`. The bracket direction `[` or `]` dictates the flow of data.

#### `launch/display.launch.py`
A lightweight utility. It does **not** launch Gazebo. Instead, it launches `robot_state_publisher`, RViz, and `joint_state_publisher_gui`. This allows a developer to manually manipulate the robot's joints using a GUI to ensure the URDF limits and TF tree are correctly defined before ever running a physics simulation.

---

### 1.2. `agrios_control` (Sensor Degradation & Data Prep)
Simulators like Gazebo provide perfect data. If you write a navigation stack that expects perfect GPS, it will instantly fail in the real world. This package exists to "ruin" the perfect data to test the robustness of the localization stack.

#### `agrios_control/rtk_sim.py`
This Python script simulates an **ArduSimple ZED-F9P RTK receiver**. It takes the perfect `/navsat` coordinate and adds configurable Gaussian noise to simulate the real-world drift of an RTK fix.

```python
        # metres -> degrees, small-angle approximation
        lat0 = self.last_fix.latitude
        dlat = random.gauss(0.0, self.sh) / 111320.0
        dlon = random.gauss(0.0, self.sh) / (111320.0 * math.cos(math.radians(lat0)))

        fix.latitude = lat0 + dlat
        fix.longitude = self.last_fix.longitude + dlon
```
Here, `self.sh` is the horizontal sigma (e.g., 0.015m for RTK fixed). It applies this noise to both latitude and longitude, scaling it properly based on the Earth's radius at that specific latitude, and then publishes this degraded data to `/rtk/fix`.

#### `agrios_control/odom_cov_relay.py`
Raw odometry from wheeled plugins often lacks properly tuned covariance matrices. A Kalman filter relies on covariance to know how much to trust a sensor. This node likely acts as a relay, injecting realistic covariance values into the `/odom` messages before they hit the filter.

---

### 1.3. `agrios_bringup` (State Estimation Orchestration)
This package ties everything together by running the high-level localization nodes.

#### `launch/localization.launch.py`
This launch file starts the custom noise simulators from `agrios_control` and then initializes the `robot_localization` suite—specifically, the Dual Extended Kalman Filter (EKF).

---

## 🧠 2. The Localization Stack: Dual EKF Theory

Agrios utilizes a **Dual Extended Kalman Filter (EKF)** architecture. This is the industry standard for outdoor robots that possess both continuous relative sensors (wheels, IMU) and absolute, potentially discontinuous sensors (GPS).

### Why Dual EKF?
If you feed GPS directly into your local control loop, a sudden GPS glitch (e.g., driving under a tree) will cause the robot's perceived position to instantly "teleport" 3 meters to the left. The local planner will react violently, jerking the steering to correct it. 
To solve this, we run two separate filters:
1.  **Local Filter (`odom` frame):** Perfectly smooth, continuous, but slowly drifts over time. Used by local planners.
2.  **Global Filter (`map` frame):** Absolute, corrects drift using GPS, but can jump. Used by global path planners.

### 2.1. `ekf_local` (The Local Filter)
Configured by `config/ekf_local.yaml`. It fuses wheel odometry and IMU data to publish the `odom` -> `base_footprint` TF transform.

Look at how the sensors are configured in the YAML:
```yaml
    # ---- wheel odometry (velocity only) ----
    odom0: /odom_cov
    # x  y  z | roll pitch yaw | vx vy vz | vroll vpitch vyaw | ax ay az
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,  # fusing Vx, Vy
                   false, false, true,   # fusing Vyaw rate
                   false, false, false]
```
The local filter ignores the absolute `X, Y` position of the wheel odometry, and only fuses the *velocities* (`vx`, `vy`, and `vyaw`). This prevents the EKF from becoming overconfident in a drifting absolute position.

### 2.2. `navsat_transform_node`
GPS provides spherical Earth coordinates (Latitude, Longitude). ROS requires a flat Cartesian plane (X, Y in meters). 
Configured by `navsat_transform.yaml`, this node listens to the RTK GPS (`/rtk/fix`), the absolute heading from the IMU (`/rtk/heading`), and the current odometry. It calculates the Earth's curvature at the robot's location and projects the lat/lon coordinates onto a local 2D grid, publishing standard odometry messages representing the absolute global position.

### 2.3. `ekf_global` (The Global Filter)
Configured by `config/ekf_global.yaml`. It fuses everything the local filter uses, **PLUS** the absolute position provided by the `navsat_transform_node`.

```yaml
    # ---- GPS (absolute position only) ----
    odom1: /odometry/gps
    odom1_config: [true,  true,  false,  # Fusing absolute X, Y
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]
```
By fusing the absolute `X` and `Y`, this filter calculates the drift of the `odom` frame and publishes a `map` -> `odom` TF transform. 

In ROS, TF trees can only have one parent. The tree looks like this:
`map` -> `odom` -> `base_footprint`

The Local EKF updates the `odom` -> `base_footprint` link smoothly at 50Hz. The Global EKF updates the `map` -> `odom` link. This means the robot's absolute position on the map is corrected, but its local trajectory remains smooth!

---

## 🚀 3. How to Run the System

Follow these steps to spin up the entire simulation and state estimation pipeline.

1. **Build the Workspace:**
   ```bash
   cd ~/Documents/ros/agrios_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   ```

2. **Source the Workspace:**
   ```bash
   source install/setup.bash
   ```

3. **Start the Physics Simulation:**
   This opens Gazebo, loads the `agrios.urdf.xacro` (defaulting to skid-steer mode), and establishes the ROS 2 to Ignition DDS bridge.
   ```bash
   ros2 launch agrios_description sim.launch.py
   ```

4. **Start the Localization Stack:**
   In a new terminal (remember to `source install/setup.bash`), launch the state estimator.
   ```bash
   ros2 launch agrios_bringup localization.launch.py
   ```
   
   This single command will:
   * Start `rtk_sim` to begin degrading the perfect Gazebo GPS data.
   * Start `odom_cov_relay` to inject covariance into the wheels.
   * Start `ekf_local` to publish smooth `odom` TF.
   * Start `navsat_transform_node` to convert the degraded lat/lon to map coordinates.
   * Start `ekf_global` to publish the absolute `map` TF.

You can now open RViz, set your fixed frame to `map`, and observe the Agrios robot navigating a simulated world with an industry-grade localization backend.
