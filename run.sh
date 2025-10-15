colcon build --cmake-args -DBUILD_TESTING=ON
. ./install/setup.sh
ros2 launch bringup new.launch.py rviz:=true
