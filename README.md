## How to use 
1. Clone this repository into your catkin_ws/src

2. Clone the following repositories into your catkin_ws/src
```
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git (Simulator for Velodyne LiDARs)
git clone https://github.com/praveen-palanisamy/multiple-object-tracking-lidar.git (Point cloud object tracker)


If mapping needs to be done then I usually use the Clearpath Husky packages from here http://wiki.ros.org/Robots/Husky

The mapping.launch file and teleop_node.py files are built around those.

```
3. Build your workspace (This might take a while to build all the laser plugins)
```
catkin_make
```
4. Launch the simulation
```
roslaunch lidar_fusion_robot gazebo.launch 
```

The object detection and tracking isn't working yet but to visualize them in RVIZ you can add the published Marker Array to the list of displayed topics.

