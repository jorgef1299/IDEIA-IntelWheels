# IDEIA-IntelWheels
This is a ROS2 Foxy package that reads the orientation and the point cloud from the StereoLabs ZED 2 depth camera and publishes this data.
The subscriber grabs this data and processes it, evaluating in the end the danger of each situation.

## Install Package
```
$ source /opt/ros/foxy/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/jorgef1299/IDEIA-IntelWheels.git
$ cd ..
$ colcon build
$ source install/setup.bash
```
## Run package
### Set Parameters file
```
$ nano ~/ros2_ws/src/pitfall_detector/config/params.yaml
```
### Run ZED 2 Publisher
```
$ ros2 run pitfall_detector zed2_pub --ros-args --params-file ~/ros2_ws/src/pitfall_detector/config/params.yaml
```
### Run Subscriber
```
$ ros2 run pitfall_detector Depth_Subscriber --ros-args --params-file ~/ros2_ws/src/pitfall_detector/config/params.yaml
```
