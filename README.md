# Piper Robot ROS 2 Project (Humble)

This repository contains the ROS 2 packages for simulating and controlling the Piper robotic arm.

## Overview

This project allows you to:
* Simulate the Piper arm (with optional gripper) in Gazebo.
* Control the arm manually using RViz and MoveIt2.
* Execute automated sequences of movements using a C++ script.

## Prerequisites

  * Ubuntu 22.04 LTS
  * ROS 2 Humble Hawksbill
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```
  * Open Manipulator X driver
    ```bash
    sudo apt install \
      ros-humble-open-manipulator-x-controller \
      ros-humble-open-manipulator-x-msgs
    ```
  * Development tools
    ```bash
    sudo apt install python3-colcon-common-extensions git
    ```
  * Python dependencies
    ```bash
    pip install numpy scipy matplotlib
    ```

## Installation and Setup
The installation of the repository can be done using the command prompt as follows:

1.  **Install Basic Tools and Support Files**
    ```bash
    sudo apt update
    sudo apt install git python3-pip can-utils ethtool
    ```
    
    ```bash
    pip3 install piper_sdk
    ```
    
2.  **Update your ~/.bashrc environment**  
    **Method 1 (Recommended):**  
    Copy and paste this entire block into your terminal once:

    ```bash
    cat << 'EOF' >> ~/.bashrc
    # --- Piper ROS Workspace ---
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source ~/piper_ros/install/setup.bash

    # --- Piper Aliases ---
    alias srcs='source install/setup.bash'
    alias sr='source venv/bin/activate'
    alias co='conda activate bnet'

    # Launch Files
    alias lau2='ros2 launch piper start_single_piper.launch.py'
    alias el2='ros2 launch piper_evaluation piper_evaluation.launch.py'
    alias ikp='ros2 launch piper_ik_to_controller piper_ik_position.launch.py'
    alias ikv='ros2 launch piper_ik_to_controller piper_ik.launch.py'

    # Web Bridges
    alias web3='ros2 run piper_teleop_bridge websocket_bridge_node_3'
    alias web4='ros2 run piper_teleop_bridge websocket_bridge_node_4'

    # Motion Command: Home Pose
    alias po='ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"piper_single\"}, name: [\"joint1\", \"joint2\",\"joint3\",\"joint4\",\"joint5\",\"joint6\",\"joint7\"], position: [0.0,0.50,-0.50,0.0,0.0,0.0,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}" --once'
    # Motion Command: Shutdown PiPER
    alias po2='ros2 service call /enable_srv piper_msgs/srv/Enable "{enable_request: false}"; ros2 topic pub /enable_flag std_msgs/msg/Bool "{data: false}" --once'
    EOF
    ```

    **Method 2**
    ```bash
    # 1. Setup workspace sourcing
    echo "" >> ~/.bashrc
    echo "# --- Piper ROS Workspace ---" >> ~/.bashrc
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    echo "source ~/piper_ros/install/setup.bash" >> ~/.bashrc

    # 2. Basic environment aliases
    echo "" >> ~/.bashrc
    echo "# --- Piper Aliases ---" >> ~/.bashrc
    echo "alias srcs='source install/setup.bash'" >> ~/.bashrc
    echo "alias sr='source venv/bin/activate'" >> ~/.bashrc
    echo "alias co='conda activate bnet'" >> ~/.bashrc

    # 3. Launch aliases
    echo "alias lau2='ros2 launch piper start_single_piper.launch.py'" >> ~/.bashrc
    echo "alias el2='ros2 launch piper_evaluation piper_evaluation.launch.py'" >> ~/.bashrc
    echo "alias ikp='ros2 launch piper_ik_to_controller piper_ik_position.launch.py'" >> ~/.bashrc
    echo "alias ikv='ros2 launch piper_ik_to_controller piper_ik.launch.py'" >> ~/.bashrc

    # 4. Web Bridge aliases
    echo "alias web3='ros2 run piper_teleop_bridge websocket_bridge_node_3'" >> ~/.bashrc
    echo "alias web4='ros2 run piper_teleop_bridge websocket_bridge_node_4'" >> ~/.bashrc

    # 5. Complex ROS 2 Publisher Aliases (Fixed quoting)
    # 'po' - Moves to position 1
    echo "alias po='ros2 topic pub /joint_states sensor_msgs/msg/JointState \"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \\\"piper_single\\\"}, name:[\\\"joint1\\\",\\\"joint2\\\",\\\"joint3\\\",\\\"joint4\\\",\\\"joint5\\\",\\\"joint6\\\",\\\"joint7\\\"], position: [0.0,0.50,-0.50,0.0,0.0,0.0,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}\" --once'" >> ~/.bashrc

    # 'po2' - Moves to position 2 (Derived from your input text)
    echo "alias po2='ros2 topic pub /joint_states sensor_msgs/msg/JointState \"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \\\"piper_single\\\"}, name: [\\\"joint1\\\", \\\"joint2\\\",\\\"joint3\\\",\\\"joint4\\\",\\\"joint5\\\",\\\"joint6\\\",\\\"joint7\\\"], position: [0.0,0.0,0.0,0.0,0.3,-0.0,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}\" --once'" >> ~/.bashrc

    # 'piper_disable' - Disables the robot (Service call + Flag pub)
    echo "alias piper_disable='ros2 service call /enable_srv piper_msgs/srv/Enable \"{enable_request: false}\"; ros2 topic pub /enable_flag std_msgs/msg/Bool \"{data: false}\" --once'" >> ~/.bashrc
    ```
    
    **Save and Exit.**
    
    ```bash
    source ~/.bashrc
    ```

4.  **Clone the Repository**
    
    ```bash
    git clone https://github.com/cake-lab/AR-PiPER.git piper_ros
    ```

5.  **Install Required Dependencies**
    
    **Navigate into the workspace**
    ```bash
    cd ~/piper_ros
    ```
    *(Note: We explicitly name the target folder `piper_ros` during cloning.)*
    
    **Install MoveIt2**
    ```bash
    sudo apt update
    sudo apt install ros-humble-moveit*
    ```
    
    **Install Gazebo and Required Plugins**
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros ros-humble-gazebo-plugins
    ```
    
    **Install Common Controllers**
    ```bash
    sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
    echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
    ```
    
    **Final Dependency Check**
    ```bash
    sudo apt update && rosdep init # Initialize rosdep if you haven't already
    rosdep update 
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "warehouse_ros_mongo"
    #rosdep install --from-paths src --ignore-src -r -y 
    ```
    
6.  **Build the Workspace:** Build all the packages using `colcon`:
    ```bash
    cd ~/piper_ros
    colcon build  
    ```
    
7.  **Source the setup files (add to `~/.bashrc`)**

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/piper_ros/install/setup.bash"  >> ~/.bashrc
    source ~/.bashrc
    ```

## Interfacing the PiPER Robotic Arm with your system.

1.  **Connect the PiPER Arm to your system via USB-CAN**
2. **Activate the CAN-BUS Port**  

   **Follow the Procedure Below to find your CAN-BUS Port (For 1st Time Users Only)**  
    ```bash
    cd ~/piper_ros
    bash find_all_can_port.sh
    ```
    
    *You will receive a message as follows if the installation is done correctly: *
    
    ```bash
    Both ethtool and can-utils are installed.
    Interface can0 is connected to USB port 3-1.4:1.0
    ```
    
    *If there are multiple modules, the the output will look like: *
    
    ```bash
    Both ethtool and can-utils are installed.
    Interface can0 is connected to USB port 3-1.4:1.0
    Interface can1 is connected to USB port 3-1.1:1.0
    ```
    
    **Activate the CAN-BUS Port**
    
    ```bash
    bash can_activate.sh can0 1000000 "3-1.4:1.0" # replace this with your port-code
    ```

## Running the Teleop Pipeline

Open three to four terminals (or tabs). In each terminal, first source your environment:

```bash
cd ~/piper_ros/
colcon build
source install/setup.bash # or simply run srcs
```

The aliases that have been added into your "~/.bashrc" file are:

    alias srcs='source install/setup.bash'  
    alias sr='source venv/bin/activate'  
    alias lau2='ros2 launch piper start_single_piper.launch.py'  
    alias ikp='ros2 launch piper_ik_to_controller piper_ik_position.launch.py'  

    alias ikv='ros2 launch piper_ik_to_controller piper_ik.launch.py'  
    alias po='ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'], position: [0.0,0.50,-0.50,0.0,0.0,0.0,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}" --once  
    alias el2='ros2 launch piper_evaluation piper_evaluation.launch.py'    
    
    
    alias po2='# call service
    ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ false\ 
    # pub topic
    ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ false\ --once'
    
    alias el2='ros2 launch piper_evaluation piper_evaluation.launch.py'  
    alias web3='ros2 run piper_teleop_bridge websocket_bridge_node_3'  
    alias web4='ros2 run piper_teleop_bridge websocket_bridge_node_4'  

| Alias  | Command                                                                     | Purpose                                                |
| :----- | :-------------------------------------------------------------------------- | :----------------------------------------------------- |
| `srcs` | `source install/setup.bash`                                                 | Source the local OMX workspace setup files             |
| `lau2` | `ros2 launch piper start_single_piper.launch.py`                            | Start the PiPER hardware controller                    |
| `ikp`  | `ros2 launch piper_ik_to_controller piper_ik_position.launch.py`            | Launch the position control ROS2 nodes                 |
| `po`   | `ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'],position: [0.0,0.50,-0.50,0.0,0.0,0.0,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}" --once` | Run basic robot position control                       |
| `el2`  | `ros2 launch piper_evaluation piper_evaluation.launch.py`                   | Launch evaluation nodes for testing robot behavior     |
| `po2`  | `# call service`                                                            | Cutoff power to all motors and shut down the robot     |
|        |`ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ false\`|                                                        |
|        |`# pub topic`                                                                |                                                        |
|        |`ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ false\ --once'`        |                                                        |
| `web3` | `ros2 run piper_teleop_bridge websocket_bridge_node_3`                      | Start the WebSocket bridge node for CV Control         |
| `web4` | `ros2 run piper_teleop_bridge websocket_bridge_node_4`                      | Start the WebSocket bridge node for ARCore App Control |

Then, run the following commands in separate terminals:

| Terminal | Command                                                                  | Purpose                                                 |
| :------- | :----------------------------------------------------------------------- | :------------------------------------------------------ |
| 1        | `ros2 launch piper start_single_piper.launch.py` or `lau2`               | Start the PiPER hardware controller                     |
| 2        | `ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'],position: [0.0,0.50,-0.50,0.0,0.0,0.0,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}" --once` or `po`  | Run basic robot position control                       |
|          | `ros2 launch piper_ik_to_controller piper_ik_position.launch.py` or `ikp`| Launch the position control ROS2 nodes                  |
| 3        | `ros2 run piper_teleop_bridge websocket_bridge_node_3` or `web3`         | Start the WebSocket bridge node for CV Control          |
|          | `ros2 run piper_teleop_bridge websocket_bridge_node_4` or `web4`         | Start the WebSocket bridge node for ARCore App Control  |

Feel free to open issues or submit pull requests for improvements!
