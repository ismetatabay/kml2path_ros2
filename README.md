# kml2path_ros2
Package that converts google's kml files to local coordinates on ROS 2.

## KML file visualization on Google Earth.
<img src="./images/kml_gearth.jpg" width="480px">

## Package output on rviz (nav_msgs/Path).
<img src="./images/kml_rviz2.jpg" width="480px">

## Requirement and Build
You need  [libkml](https://github.com/google/libkml) for parse kml.
You need  [GeographicLib](https://geographiclib.sourceforge.io/C++/doc/install.html) for transform lat lon alt to local coordinates.

Clone repository into your workspace.
```
cd ~/ros2_ws/src
git clone https://github.com/ismetatabay/kml2path_ros2.git
```
build (ros2 terminal)
```
cd ~/ros2_ws
colcon build 
```
## Usage
Firstly, source your local workspace
```
source ~/ros2_ws/install/setup.bash
```
Launch 
```
ros2 launch kml2path_ros2 kml2path

```
