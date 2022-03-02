# remaro_uw_sim
ROS and gazebo-based underwater simulation for the REMARO project.
Currently, the simulator includes an underwater environment with a bop panel (from uuv simulator), a BlueROV2 robot and a RexROV robot, both controlled with a joystick.

#TODO: 

- generate a 3D environment for pipeline inspection.
- integrate more robots, if required for the project.
- integrate a controller for the robot, either autonomous or teleoperated.

# 1. Running #
Here we detail the different ways of simulating an underwater robot with gazebo, airsim or both. The ROS package remaro_uw_sim provides launchers that make the work easier for you.

## 1.1 standalone Gazebo simulation
You can either use any of the launch files that already does the job for you of launching both the world environment and the robot, or you can manually do it yourself

- remaro_uw_sim launchers:
    - `$ roslaunch uwsim bluerov2_joy_simulator.launch`

- Manually launching a world and spawning the robot:
    1. Launch gazebo with the UUV Simulator environment 
       - `$ roslaunch uuv_gazebo_worlds subsea_bop_panel.launch`
    2. Spawn the robot
       - `$ roslaunch bluerov2_description upload_bluerov2.launch`  

## 1.2 AirSim & ROS

Under AirSim you can find a ros workspace folder with ros packages that will bridge between your AirSim simulation and ROS.

      ```
      cd PATH_TO/AirSim/ros
      catkin build 
      source ./devel/setup.bash
      ```

## 1.3 AirSim visuals with Gazebo physics

1. First, run the AirSim simulator:

   - Open the Unreal Engine launcher:
         ```
         cd PATH_TO/UE4Launcher
         npm start
         ```
   - Run your project. Remember to set the `GameMode Override` under the tab `World Settings` to `AirSimGameMode`.

   - Play the simulation.

2. Run the Gazebo simulation with your Gazebo model.

      ```
      roslaunch uwsim gazebo_backend.launch rov_model:=rexrov2
      ```
3. Run the GazeboDrone plugin.
   - First, you will need to modify the `settings.json` file to indicate that you will use an external physics engine. Do it by adding the line : `"PhysicsEngineName":"ExternalPhysicsEngine"`.
   - Run the plugin
      ```
      cd PATH_TO/AirSim/GazeboDrone/build
      ./GazeboDrone
      ```
4. Get ROS topics from AirSim
   - Note that the AirSim ros nodes are in a different workspace. Therefore, you will first need to source it:
      ```
      cd PATH_TO/AirSim/ros
      catkin build 
      source ./devel/setup.bash
      ```
   - And then run the airsim ROS packages:
      ```
      roslaunch airsim_ros_pkgs airsim_node.launch
      ```

   
# 2. Installation     

To setup remaro_uw_sim in our computer, we will first install all the requirements, and afterwards install the package itself.

## 2.1. Requirements

### 2.1.1 ROS, Gazebo and UUV
- [ros-\*-desktop-full](http://wiki.ros.org/ROS/Installation)
  - currently tested on ROS mellodic and Ubuntu 18.04
     - `$ sudo apt install ros-melodic-desktop-full`
- [uuv simulator](https://uuvsimulator.github.io/):
   - You can directly install it with:
      - `$ sudo apt install ros-melodic-uuv-simulator`
   - Or manually install the packages in your catkin workspace source:
      ```
      cd ~/catkin_ws/src
      git clone https://github.com/olayasturias/uuv_simulator.git
      ```
   
- [RexROV2 package](https://github.com/uuvsimulator/rexrov2) Installation. Clone it in your catkin workspace source folder with:

      ```
      cd ~/catkin_ws/src
      git clone https://github.com/uuvsimulator/rexrov2.git
      ```

   Build all your packages with

   ```
   cd ~/catkin_ws
   catkin build
   ```
### 2.1.2 Unreal Engine
- [Unreal Engine](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/) a photorealistic rendering engine. 
Follow the instructions in the link to create an epic games account. AirSim only supports Unreal >= 4.25. Clone and build it: 

      ```
      git clone -b 4.25 git@github.com:EpicGames/UnrealEngine.git
      cd UnrealEngine
      ./Setup.sh
      ./GenerateProjectFiles.sh
      make
      ```
   - [Epic Games Launcher](https://www.epicgames.com/store/en-US/download). If you want to get environments or plugins for your unreal engine from the [marketplace](https://www.unrealengine.com/marketplace/en-US/store), you will need to have the Epic games launcher installed in your PC. While straightforward for Windows, if you are using Ubuntu you will need the unofficial version [UE4Launcher](https://github.com/nmrugg/UE4Launcher). 

      ```
      git clone https://github.com/nmrugg/UE4Launcher.git
      cd UE4Launcher
      npm i
      npm start
      ```

### 2.1.3 AirSim
- [GCC 8](https://askubuntu.com/questions/1028601/install-gcc-8-only-on-ubuntu-18-04). The AirSim plugin GazeboDrone requires to have AirSim compiled with GCC 8. If you are using Ubuntu 18.08, you probably have GCC 7. Both compilers can coexist, we will just install and set GCC 8 as default:

      ```
      sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7
      sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8
      ```
   - You can check your gcc current version by typing in your terminal ` g++ --version`
   - If you want to change again the defaults you can do so by running  ` sudo update-alternatives --config gcc` and choosing your desired default version in the prompt.
- [AirSim](https://microsoft.github.io/AirSim/build_linux/). AirSim is a simulator for drones, cars and more, built on Unreal Engine. It is open-source, cross platform, and supports software-in-the-loop simulation with popular flight controllers such as PX4 & ArduPilot and hardware-in-loop with PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped into any Unreal environment. 
Clone and build it:

      ```
      git clone https://github.com/Microsoft/AirSim.git
      cd AirSim
      ./clean.sh
      ./setup.sh
      ./build.sh --gcc
      ```
   - [GazeboDrone](https://microsoft.github.io/AirSim/gazebo_drone/) allows connecting a gazebo drone to the AirSim drone, using the gazebo drone as a flight dynamic model (FDM) and AirSim to generate environmental sensor data. It can be used for Multicopters, Fixed-wings or any other vehicle. Inside of your AirSim folder:

      ```
      cd GazeboDrone
      mkdir build && cd build
      cmake -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8 ..
      make
      ```  


## 2.2 Installation

Install the ROS packages for remaro_uw_sim as follows:

 1. Go to your ROS package source directory:
    - `$ cd catkin_ws/src`
 2. Clone this project.
    - `$ git clone https://github.com/olayasturias/remaro_uw_sim`
 3. Build and install it:
      ```
      cd ~/catkin_ws
      catkin build remaro_uw_sim
      ```