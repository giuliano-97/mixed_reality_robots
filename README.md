# Mixed reality robots

![Manipulation demo](/docs/mr_robots.gif)

This project aims to showcase how mixed reality devices could be used to simplify the interaction between humans and robots using MRTK and ROS.

The instructions below explain how to setup the ROS/Unity environment and try the demos in Holographic Remoting mode on a Microsoft Hololens.

One of the goals for future iterations of this project would be to add support for Android devices, so that anyone with a smartphone and laptop could try it.

## ROS setup - Ubuntu
- You will need either a VM with Ubuntu 16.04 or a separate machine with Ubuntu 16.04 to which you can establish a network connection. In the first case, follow the instructions at  https://github.com/siemens/ros-sharp/wiki/User_Inst_UbuntuOnOracleVM to install Ubuntu on Oracle VM VirtualBox.
- In your Ubuntu machine install ROS Kinetic (follow the instructions at http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Once you have successfully installed ROS, fire up a terminal and execute the following command to install the necessary ROS dependencies:
```bash
sudo apt-get install ros-kinetic-rosbridge-server ros-kinetic-moveit ros-kinetic-panda-moveit-config ros-kinetic-moveit-visual-tools ros-kinetic-move-base ros-kinetic-gmapping ros-kinetic-franka-description ros-kinetic-lms1xx
```
- Create a catkin workpace in your home directory
```bash
mkdir -p ~/ws_mrrobots/src
cd ws_mrrobots
catkin_make
```
- Clone the repository of the project:
```bash
cd ~ && git clone https://gitlab.ethz.ch/albanesg/3d-vision-project
```
- Copy the content of the ROS folder in the repository in the **src** directory of the catkin workspace
```bash
cp -R ~/3d-vision-project/ROS/* ~/ws_mrrobots/src/
```
- Finally, run the following commands to build the packages
```bash
cd ~/ws_mrrobots 
source ./devel/setup.bash
catkin_make
``` 
## Unity/Hololens setup 
A video was created to aid the next steps in setting up the environment and running the demos. This video can be found [here](docs/demo_tutorial.mp4)
- You will need a machine with Windows 10 on which you are going to install the Unity game engine. The assets in this repository are guaranteed to be compatible with Unity 2019.3.*.
- Clone the repository
- Create a new Unity Project for 3D
- Go to "File" -> "Build Settings"
- Switch to "Universal Windows Platform" with the following Settings:
    - Target Device: "Any Device"
    - Architecture: "x64"
    - Build Type: "D3D Project"
    - Target SDK Version: "Latest installed"
    - Build and Run on: "Local Machine"
    - Build configuration: "Release"
- Go to "Edit" -> "Project Settings" -> "Player" -> "XR Settings"
    - Activate "Virtual Reality Supported"
    - Under "Virtual Reality SDKs" add "Windows Mixed Reality"
    - Set it to "16-bit depth"
    - Enable "Enable Depth Buffer Sharing"
- Copy the content of the "Assets" folder into the projects' "Assets" folder
- In the MRTK Pop-up press "Apply" -**Do this whenever the pop-up shows up!**
- Go to "Scenes" and open "Robot Manipulation"

## Running the demos - ROS 
Before running any of the available robot simulations, fire up a terminal in Ubuntu and run the following commands:
```bash
cd ws_mrrobots && source ./devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Manipulation with obstacle avoidance and autonomous navigation:
- Fire up a new terminal and run the following commands to start the simulation
of the Panda manipulator with obstacle avoidance enabled:
```bash
cd ws_mrrobots && ./devel/setup.bash
roslaunch panda_unity_simulation manip_obstacles_sim.launch 
```
- In a separate terminal run the following commands to start the simulation of the
Jackal robot with the navigation stack:
```bash
cd ws_mrrobots && ./devel/setup.bash
roslaunch jackal_unity_simulation navigation_sim.launch
```

### Manipulation in Follow Mode:
- Fire up a new terminal and run the following commands to start the simulation of
the Panda manipulator in follow mode:
```bash
cd ws_mrrobots && ./devel/setup.bash
roslaunch panda_unity_simulation follow_object.launch
```

## Running the demos - UNITY
- Start up the Hololens and open the "Holographic Emulation" Application
- In Unity go to "Window" -> "XR" -> "Holographic Emulation"
- Choose "Remote to Device" and enter the IP-Adress of the Hololens. Choose the correct Hololens version in the dropdown.
- Press "Connect". The Hololens must be in the same network. The button will turn green and nothing should be displayed in the hololens anymore.
- Now on to the demos
- **Manipulation with obstacle avoidance and autonomous navigation**:
    - Open the scene "ObstaclesAndNavigation" in folder "Scenes" in the Assets menu
    - In the scene object list, select "RosConnectorManipulation". In the inspector, in the component named "Ros Connector" enter the IP address of the Ubuntu machine where the ROS will be running
    - Now start the app - press the Play button
    - Move Jackal to the floor with the grab gesture and press "Start Connection". Make sure the robot is on the floor, as the robot can not be moved anymore after pressing this button! 
    - Panda and its target can be moved with the grab gesture as well.
    - By pressing "Set Navigation Target", a green sphere can be seen. By tapping once, the navigation target is set. This target can not be moved but instead be placed again. By pressing "Execute Navigation", the robot will try to reach the target. 
    - By pressing "Manipulation Planning", the robot will try to find a path to the goal. If no path is found, a red error text is displaced (it takes a while until the text is displayed). If a path is found, the robot starts moving upon pressing "Manipulation Execution".
- **Manipulation in Follow Mode**:
    - Open the scene "FollowManipulation" in folder "Scenes" in the Assets menu
    - In the scene object list, select "RosConnectorFollow". In the inspector, in the component named "Ros Connector" enter the IP address of the Ubuntu machine where the ROS will be running
    - Move the robot with the grab gesture, as well as the orange target. 
    - Press "Follow Mode" to toggle this mode. If it is enabled, the robot will skip the other two buttons and always instantly try to follow the target. Press again to stop this function.

## Used Packages

| Name | Version Number | Source | Comments |
|---|---|---|---|
| Microsoft Mixed Reality Toolkit Unity Foundation | 2.4.0 | https://github.com/Microsoft/MixedRealityToolkit-Unity/releases | - |
| Ros Sharp | 1.6 | https://github.com/siemens/ros-sharp | After the commits in April the package stopped working. If it still isn't fixed, try to get the version from March. |
| ROS | Kinetic | http://wiki.ros.org/ | - |
| gmapping | Kinetic | http://wiki.ros.org/gmapping | - |
| move_base | Kinetic | http://wiki.ros.org/move_base | - |
| Moveit | 1 - Kinetic | https://moveit.ros.org/ | The version for Ubuntu 16.04 was used. |