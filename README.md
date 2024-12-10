# ros2-humble-docker-setup
A step-by-step guide to setting up a ROS2 Humble workspace in Docker, including rosbridge_websocket and ROSbot simulation.

# Setting Up ROS2 Humble in Docker and Related Tools

## Step 1: Install ROS2 Humble in a Docker Environment

1. Open Visual Studio Code (VSCode).
2. Install the **Remote Development** extension.
3. Open the menu in the bottom-left corner (`><`) and select **New Dev Container**.
4. Search for `ros2` and select the **ROS2 Workspace BrunoB81HK** option.
5. Once set up, validate the ROS2 installation by running the following command in the terminal:

   "ls /opt/ros"

   You should see the available ROS2 distributions, including `humble`.

---

## Step 2: Install `rosbridge_websocket`

1. **Install the `rosbridge_server` Package**:

   "sudo apt update && sudo apt install -y ros-humble-rosbridge-server"

2. **Verify the Installation**:

   "ros2 pkg list | grep rosbridge_server"

   Ensure `rosbridge_server` appears in the output.

3. **Launch the WebSocket Server**:

   "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

   This starts the WebSocket server on the default port (9090).

4. **Optional - Run Manually**: You can also start the server directly without a launch file:

   "ros2 run rosbridge_server rosbridge_websocket"

---

## Step 3: Install and Simulate ROSbot

1. **Set Up the ROSbot Gazebo Simulation**:  
   Clone and build the repository:

   "git clone https://github.com/husarion/rosbot_ros.git && cd rosbot_ros && colcon build"

2. **Run the Simulation**:  
   Launch the ROSbot simulation in Gazebo:

   "source install/setup.bash && ros2 launch rosbot_gazebo simulation.launch.py"

3. **Simulate Multiple Robots**:  
   To launch multiple robots in the simulation:

   "ros2 launch rosbot_gazebo simulation.launch.py robots:=\"robot1={x: 0.0, y: 0.0, yaw: 0.0}; robot2={x: 2.0, y: 0.0, yaw: 1.57};\""
