# Installation
Create a ros2_ws

clone the following into src folder: 

crazyflie_hardware -b sitl_radio
crazyflie_interfaces
ros-objecttracker
this repo

Then colcon Build


# Test

3 Terminals with ros sourced
1 Terminal for the firmware

1. Startup the firmware
2. `ros2 launch crazyflie_hardware_bringup hardware.launch.py` 
3. `ros2 run mock_sim mock_sim`

You should now see messages from the firmware, hardware launch is still pretty empty

4. Add crazyflie to gateway 

```
ros2 service call /crazyflie_hardware_gateway/add_crazyflie crazyflie_hardware_gateway_interfaces/srv/AddCrazyflie "channel: 80
id: 231
initial_position:
  x: 0.0
  y: 0.0
  z: 0.0
type: default" 
```

This will now startup a ros2 node exactly like when used with real crazyflie. 
You can set Parameters, start log blocks, send commands

# IMPORTANT
There is no simulation yet!!
If you send takeoff crazyflie will stay at 0,0,0
The simulation connection to webots still needs to be implementet. 

Mock just sends "i am stationary on the ground" data


# Building the firmware

clone the firmware with branch dev-crazysim

git submodule update --init --recursive vendor

sudo apt install cmake build-essential
pip install Jinja2
sudo apt install python-is-python3

mabye: remove gz from CMakeLists in sitl_make

# Building

From main repo: 

make sitl_defconfig
make PLATFORM=sitl

# Executing

build/cf2