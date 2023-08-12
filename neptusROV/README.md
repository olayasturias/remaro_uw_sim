# Dependencies

- Neptus (LSTS Toolchain) installation and usage instructions are available [here](github.com/LSTS/neptus/wiki).
- Dune (LSTS Toolchain) installation and usage instructions are available [here](github.com/LSTS/dune/wiki).
- [imc_ros_bridge](https://github.com/smarc-project/imc_ros_bridge)

## Building

A simple `colcon build --packages-select neptus_interface` 

# UNavSim settings
Find the UnavSim settings file inside the folder `Documents/AirSim` (note that UNavSim is developed on top of AirSim).
Inside your `settings.json` file you need to add this line:  
`"PhysicsEngineName":"ExternalPhysicsEngine"`

# Run
First run the AirSim simulator and Neptus and then execute the ros node bridging them:

```
ros2 run neptus_interface neptus2unavsim
```