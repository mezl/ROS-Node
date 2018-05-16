#!/bin/bash
#This program will create symb link of /dev/imu and /dev/hokuyo
#source ~/catkin_ws/devel/setup.bash
ACM0=`udevadm info -a all -n /dev/ttyACM0 --attribute-walk|grep manu|head -n1`
ACM1=`udevadm info -a all -n /dev/ttyACM1 --attribute-walk|grep manu|head -n1`
if [[ $ACM0 = *"Hokuyo"* ]]; then
    echo "Hokuyo is ttyACM0"
    HOKUYO_PORT="/dev/ttyACM0"
    sudo ln -sf $HOKUYO_PORT /dev/hokuyo
elif [[ $ACM0 = *"Arduino"* ]]; then
    echo "IMU is ttyACM0"
    IMU_PORT="/dev/ttyACM0"
    sudo ln -sf $IMU_PORT /dev/imu
fi
if [[ $ACM1 = *"Hokuyo"* ]]; then
    echo "Hokuyo is ttyACM1"
    HOKUYO_PORT="/dev/ttyACM1"
    sudo ln -sf $HOKUYO_PORT /dev/hokuyo
elif [[ $ACM1 = *"Arduino"* ]]; then
    echo "IMU is ttyACM1"
    IMU_PORT="/dev/ttyACM1"
    sudo ln -sf $IMU_PORT /dev/imu
fi
echo "HOKUYO port is " $HOKUYO_PORT
echo "IMU port is " $IMU_PORT
