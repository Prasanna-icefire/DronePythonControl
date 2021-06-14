# DronePythonControl

These are for my reference.....
To install mavros on ubiquity image for ROS implementation, use :

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream-development mavros mavros_extras mavros_msgs test_mavros sensor_msgs  control_toolbox realtime_tools tf tf2_ros python_orocos_kdl urdf |tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
catkin build

Installing this on piOS is not possible at present!
