Polaris Hybrid Position Sensor driver
==============
This package contains a ROS-independant library to get information from the Polaris (based on https://github.com/RoverRobotics-forks/serial-ros2) and a simple ROS wrapper to send a geometry_msgs::PoseArray (visualizable on Rviz2).

### Usage
Two parameters are needed, the .rom files and the port to which the sensor is connected :
```bash
ros2 run polaris_sensor polaris_sensor_node --ros-args -p roms:=/your_rom_file_location/your_marker_tool.rom port:=/dev/ttyUSB0
```

If you have **multiple** rom files :
```bash
ros2 run polaris_sensor polaris_sensor_node --ros-args -p roms:="/dir_to/marker_1.rom,/dir_to/marker_2.rom" port:=/dev/ttyUSB0
```

If you want to get the two rom relative transformation:
```bash
ros2 run polaris_sensor polaris_sensor_double_node --ros-args -p roms:="$(rospack find polaris_sensor)/rom/marker_1.rom,"$(rospack find polaris_sensor)/rom/marker_2.rom _port:=/dev/ttyUSB0 
```
The first the rom is the target corrdinate and the second rom is the base corrdinate, then you can subscribe the topic `/polaris_sensor/targets_in_base` to get the transformation

For convenience you can also use the launch file (default single rom tracking, you can set your rom file in this launch)
```bash
ros2 launch polaris_sensor ndi_polaris_driver.launch.py
```

**Attention**: If you cannot get the permission of the usb port, you need to add the the rules file, you can follow this command:
To find out the idVendeor and idProduct number:
```bash
lsusb
```

you can get the information like that:
```bash
Bus 001 Device 007: ID 067b:2303 Prolific Technology, Inc. PL2303 Serial Port / Mobile Action MA-8910P
```

Then you need to add the rules file to /etc/udev/rules.d which file name you need to named "70-ttyusb.rules"
And you need to add this to this file:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",MODE="0666"
```

Then reinsert the ndi device

>Note: The rate is 60Hz.


Authors: Antoine Hoarau, Florian Richer
Contributors: Boyang Ti

Ported to ROS2 by: Joshua Liu