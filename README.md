# remaro_uw_sim
ROS and gazebo-based underwater simulation for the REMARO project.
Currently, the simulator includes an underwater environment with a bop panel (from uuv simulator) and a BlueROV2 robot.

#TODO: 

- generate a 3D environment for pipeline inspection.
- integrate more robots, if required for the project.
- integrate a controller for the robot, either autonomous or teleoperated.


## Getting Started

### Requirements ###
- git
- [ros-\*-desktop-full](http://wiki.ros.org/ROS/Installation)
  - currently tested on ROS mellodic and Ubuntu 18.04
- [uuv simulator](https://uuvsimulator.github.io/) Installation:
   - `$ sudo apt install ros-melodic-uuv-simulator`


### Installation ###
 1. Go to your ROS package source directory:
    - `$ cd ros_workspace_path/src`
 2. Clone this project.
    - `$ git clone https://github.com/olayasturias/remaro_uw_sim`
 3. Go back to your ROS workspace:
    - `$ cd ../`
 4. Build and install it:
    - `$ catkin build remaro_uw_sim`

## Running ##

You can either use any of the launch files that already does the job for you of launching both the world environment and the robot, or you can manually do it yourself :wink:

- remaro_uw_sim launchers:
    - `$ roslaunch remaro_uw_sim uuv_simulator.launch`

- Manually launching a world and spawning the robot:
    1. Launch gazebo with the UUV Simulator environment 
       - `$ roslaunch uuv_gazebo_worlds subsea_bop_panel.launch`
    2. Spawn the robot
       - `$ roslaunch remaro_rov_models upload_bluerov2.launch`  