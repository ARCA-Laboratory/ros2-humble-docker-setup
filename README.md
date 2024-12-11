# ROS2 Humble Setup Guide with Docker, rosbridge_websocket, and ROSbot Simulation

## Prerequisites

- **Docker** is installed and running on your system.
- **Visual Studio Code (VSCode)** is installed, along with the Remote Development extension.
- Internet access to download necessary packages.

---

## Steps to Set Up ROS2 Humble and Associated Tools

### Step 1: Set Up ROS2 Humble in Docker

1. Open Visual Studio Code (VSCode).
2. Install the **Remote Development** extension.
3. Open the bottom-left menu (`><`) in VSCode and select **New Dev Container**.
4. Search for `ros2` and select the **ROS2 Workspace BrunoB81HK** option.
5. Once the setup is complete, validate the ROS2 installation by running the following command in the terminal:

   ```bash
   ls /opt/ros
   ```

   You should see the available ROS2 distributions, including `humble`.

---

### Step 2: Install and Configure `rosbridge_websocket`

1. **Install the `rosbridge_server` Package**:

   ```bash
   sudo apt update && sudo apt install -y ros-humble-rosbridge-server
   ```

2. **Verify the Installation**:

   ```bash
   ros2 pkg list | grep rosbridge_server
   ```

   Ensure `rosbridge_server` is listed in the output.

3. **Launch the WebSocket Server**:

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

   This starts the WebSocket server on the default port (9090), allowing communication between ROS2 and external systems.

4. **Optional - Run Manually Without Launch File**:

   ```bash
   ros2 run rosbridge_server rosbridge_websocket
   ```

---

### Step 3: Install and Run ROSbot Simulation

1. **Set Up the ROSbot Simulation Workspace**:

   Clone the `rosbot_ros` repository and set up the simulation environment:

   ```bash
   mkdir -p ros2_ws
   cd ros2_ws
   git clone https://github.com/husarion/rosbot_ros src/rosbot_ros
   export HUSARION_ROS_BUILD_TYPE=simulation
   vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos
   cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers
   sudo rosdep init
   rosdep update --rosdistro $ROS_DISTRO
   rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
   colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

2. **Run the ROSbot Simulation**:

   Launch the simulation in Gazebo:

   ```bash
   source install/setup.bash
   ros2 launch rosbot_gazebo simulation.launch.py
   ```

3. **Simulate Multiple Robots**:

   To launch multiple robots in the simulation, use the following command:

   ```bash
   ros2 launch rosbot_gazebo simulation.launch.py robots:="robot1={x: 0.0, y: 0.0, yaw: 0.0}; robot2={x: 2.0, y: 0.0, yaw: 1.57};"
   ```

---

## Troubleshooting and Additional Resources

For detailed troubleshooting steps and more information, refer to the [official ROSbot documentation](https://github.com/husarion/rosbot_ros).
