#!/bin/sh

cd ~/catkin_ws &
source devel_isolated/setup.bash

# display
roscore
roslaunch laser_scan_matcher demo.launch

# Localization
roslaunch rplidar_ros rplidar.launch
./devel_isolated/imu_complementary_filter/lib/imu_complementary_filter/complementary_filter_node _use_mag:=true
sudo pigpiod
rosrun driver_mpu9250 driver_mpu9250_rpi #imu
./devel_isolated/laser_scan_matcher/lib/laser_scan_matcher/laser_scan_matcher_node _use_imu:=true



# Control
roslaunch pca9685_board pca9685_board.launch #pwm
# rostopic pub -r 10 pwm_msg pca9685_board/pwm "{channel: 11, value: 290}" # 286-310
# rostopic pub -r 10 pwm_msg pca9685_board/pwm "{channel: 15, value: 290}" # 286-310
rostopic pub -r 10 pwm_msg pca9685_board/pwm "{channel: 15,  value: 157}" # SERVO: 157-477 

# roslaunch motor_control motor_control.launch
./devel_isolated/motor_control/lib/motor_control/motor_controller_node

