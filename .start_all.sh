#!/bin/sh

cd ~/catkin_ws
source devel_isolated/setup.bash

# display
roscore &
# roslaunch laser_scan_matcher demo.launch use_rviz:=false &

# Localization
roslaunch rplidar_ros rplidar.launch &
sudo pigpiod 
rosrun driver_mpu9250 driver_mpu9250_rpi & #imu
./devel_isolated/laser_scan_matcher/lib/laser_scan_matcher/laser_scan_matcher_node _use_imu:=true &
./devel_isolated/imu_complementary_filter/lib/imu_complementary_filter/complementary_filter_node _use_mag:=true &


# Control
# roslaunch pca9685_board pca9685_board.launch & #pwm
./devel_isolated/pca9685_board/lib/pca9685_board/pca9685_board_node &

# roslaunch motor_control motor_control.launch
./devel_isolated/motor_control/lib/motor_control/motor_controller_node &


# msg examples
# rostopic pub -r 10 pwm_msg pca9685_board/pwm "{channel: 11, value: 290}" # 286-310
# rostopic pub -r 10 pwm_msg pca9685_board/pwm "{channel: 15, value: 290}" # 286-310
# rostopic pub -r 10 pwm_msg pca9685_board/pwm "{channel: 15,  value: 157}" # SERVO: 157-477 
