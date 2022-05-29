# hovercraft_bloom
# KAIST 2022 Spring Capstone Design 1

-----
## Prerequisite
1. Raspberry Pi4B packages - [Pigpio](http://abyz.me.uk/rpi/pigpio/download.html)
2. 

## Installation

```
cd ~/[your catkin_ws]/src
git clone https://github.com/mikodham/hovercraft_bloom.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

## Deploy
```
source ~/[your catkin_ws]/devel/setup.bash
catkin_make # -DCATKIN_BLACKLIST_PACKAGES="scan_to_cloud_converter"
```

# imu
sudo pigpiod
rosrun driver_mpu9250 driver_mpu9250_rpi _interrupt_gpio_pin:=14 

# lidar 
roslaunch rplidar_ros rplidar.launch

# rosbag
rosbag record *

# pwm
<!-- rosrun pwm_pca9685 pca9685_node -->
roslaunch pca9685_board pca9685_board.launch
rostopic pub -r 10 pwm_msg pca9685_board/pwm "{channel: 1, value: 290}"
# esc
rosrun control motor_control_test.py

# csm-laser_scan_matcher
```
# put the csm outside of src, because csv cannot be compiled with catkin_make
# export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/home/pi/catkin_ws/dump/csm/sm/pkg-config
#cp -r /home/pi/catkin_ws/dump/csm/deploy/include/ /home/pi/catkin_ws/install/
# source ~/catkin_ws/dump/csm/deploy/setup.bash
just catkin_make_isolated
```

---
## Developer Contact
Dhammiko Arya Gandamana - dhammikoarya@kaist.ac.kr

Sungmin Kim - sungmin203@kaist.ac.kr