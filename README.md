# Simultaneous calibration and localization
## Global calibration for UWB anchor positions with Lidar and UWB
A factor graph-based implementation for UWB anchor global calibration with LiDAR providing local poses and robot localization with UWB, which provides global positions. The imformation fusion of the robot's local pose and the global position is also included to provide a robust robot global pose.

The detailed results can be found in [our paper](https://arxiv.org/abs/2503.22272).


<!-- The convergence results of the position estimation of anchors:

<img src="./results/Estimated_coordinate.png" alt="Estimated-coordinate" width="600" height="350">


The global pose estimation of the robot:

<img src="./results/global_pose_result.png" alt="Global-path" width="600" height="350"> -->


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.
ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Follow [Ceres Solver Installation](http://ceres-solver.org/installation.html). Tested with version 2.1.0.

### 1.3. **PCL, Eigen and Others**
Follow [PCL Installation](https://pointclouds.org/downloads/#linux), and [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page) if needed. Normally, the ones with Ubuntu 20.04 are sufficient.


## 2. Build
We use a modified `A-LOAM` to estimate the local poses. Clone this repository and the `A-modified-A-LOAM`, and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/LiuxhRobotAI/Simultaneous_calibration_localization.git

    git clone https://github.com/LiuxhRobotAI/A-modified-A-LOAM.git

    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. UWB calibration
You can run the following commands to get optimal estimations of the UWB anchor's positions.
```
    roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
    rosrun global_selfcalibration global_selfcalibration_node
```
You can also run with a ROS bag. The bag data should includes the UWB distance measurements, the anchor's initial positions and the corresponding Lidar cloud points.
```
    rosbag play DATASET_FOLDER/LIDAR_UWB.bag
```

## 4. Acknowledgements
Thanks for [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion), the [updated VINS-Fusion](https://github.com/LiuxhRobotAI/VINS-Fusion), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).
For Licenses of the component codes, users are encouraged to read the material of these original projects.
