source ~/catkin_ws/devel/setup.sh
killall -9 gzserver
killall -9 gzclient
roslaunch tb3kai_spinhokuyo spin_gazebo.launch
