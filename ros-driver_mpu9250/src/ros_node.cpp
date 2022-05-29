#include "ros_node.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs_ext/accelerometer.h>
#include <sensor_msgs_ext/gyroscope.h>
#include <sensor_msgs_ext/magnetometer.h>
#include <sensor_msgs_ext/temperature.h>

#include <cmath>

// CONSTRUCTORS
ros_node::ros_node(std::shared_ptr<driver> driver, int argc, char **argv)
{
    // Initialize flags.
    ros_node::f_gyroscope_calibrating = false;

    // Create a new driver.
    ros_node::m_driver = driver;

    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mpu9250");

    // Get the node's handle.
    ros_node::m_node = std::make_shared<ros::NodeHandle>();

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_i2c_bus = private_node.param<int>("i2c_bus", 1);
    int param_i2c_address = private_node.param<int>("i2c_address", 0x68);
    int param_interrupt_pin = private_node.param<int>("interrupt_gpio_pin", 14);
    int param_gyro_dlpf_frequency = private_node.param<int>("gyro_dlpf_frequency", 0);
    int param_accel_dlpf_frequency = private_node.param<int>("accel_dlpf_frequency", 0);
    int param_gyro_fsr = private_node.param<int>("gyro_fsr", 0);
    int param_accel_fsr = private_node.param<int>("accel_fsr", 0);
    float param_max_data_rate = private_node.param<float>("max_data_rate", 8000.0F);

    // Read calibrations.
    ros_node::m_calibration_accelerometer.load(private_node, "calibration/accelerometer");
    ros_node::m_calibration_magnetometer.load(private_node, "calibration/magnetometer");
    
    // Set up data publishers.
    ros_node::m_publisher_accelerometer = ros_node::m_node->advertise<sensor_msgs_ext::accelerometer>("imu/accelerometer", 1);
    ros_node::m_publisher_gyroscope = ros_node::m_node->advertise<sensor_msgs_ext::gyroscope>("imu/gyroscope", 1);
    ros_node::m_publisher_magnetometer = ros_node::m_node->advertise<sensor_msgs_ext::magnetometer>("imu/magnetometer", 1);
    ros_node::m_publisher_temperature = ros_node::m_node->advertise<sensor_msgs_ext::temperature>("imu/temperature", 1);
    // HOVERBLOOM
    ros_node::imu_publisher = ros_node::m_node->advertise<sensor_msgs::Imu>("imu/data_raw",1);
    ros_node::mag_publisher = ros_node::m_node->advertise<sensor_msgs::MagneticField>("imu/mag",1);
    // Initialize the driver and set parameters.
    try
    {
        // Attach the data callback.
        ros_node::m_driver->set_data_callback(std::bind(&ros_node::data_callback, this, std::placeholders::_1));
        // Initialize driver.
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_i2c_bus), static_cast<unsigned int>(param_i2c_address), static_cast<unsigned int>(param_interrupt_pin));
        // Set parameters
        float data_rate = ros_node::m_driver->p_dlpf_frequencies(static_cast<driver::gyro_dlpf_frequency_type>(param_gyro_dlpf_frequency), static_cast<driver::accel_dlpf_frequency_type>(param_accel_dlpf_frequency), param_max_data_rate);
        ros_node::m_driver->p_gyro_fsr(static_cast<driver::gyro_fsr_type>(param_gyro_fsr));
        ros_node::m_driver->p_accel_fsr(static_cast<driver::accel_fsr_type>(param_accel_fsr));

        ROS_INFO_STREAM("mpu9250 driver successfully initialized on i2c bus " << param_i2c_bus << " at address 0x" << std::hex << param_i2c_address);
        ROS_INFO_STREAM("sensor data rate is " << data_rate << " hz");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
        exit(1);
    }

    // Set up services.
    ros_node::m_service_calibrate_gyroscope = ros_node::m_node->advertiseService("imu/calibrate_gyroscope", &ros_node::service_calibrate_gyroscope, this);

    // Perform initial gyroscope calibration.
    ros_node::calibrate_gyroscope(500);
}

// ROS
void ros_node::spin()
{
    // Spin.
    ros::spin();

    // Deinitialize driver.
    ros_node::deinitialize_driver();
}

// SERVICES
bool ros_node::service_calibrate_gyroscope(sensor_msgs_ext::calibrate_gyroscopeRequest& request, sensor_msgs_ext::calibrate_gyroscopeResponse& response)
{
    response.success = ros_node::calibrate_gyroscope(request.averaging_period);
    if(!response.success)
    {
        response.message = "failed to collect enough points for calibration";
    }

    return true;
}
bool ros_node::calibrate_gyroscope(uint32_t averaging_period)
{
    // Convert averaging period to duration.
    ros::Duration averaging_duration(static_cast<double>(averaging_period) / 1000.0);

    // Clear the collection window.
    ros_node::m_gyroscope_calibration_window.clear();

    // Enable the data collection.
    ros_node::f_gyroscope_calibrating = true;

    // Sleep while gyro data is collected on interrupt thread.
    averaging_duration.sleep();

    // Disable the data collection.
    ros_node::f_gyroscope_calibrating = false;

    // Check if the window contains data.
    if(ros_node::m_gyroscope_calibration_window.size() < 5)
    {
        ROS_ERROR_STREAM("gyroscope calibration failed (not enough data: " << ros_node::m_gyroscope_calibration_window.size() << " points)");
        return false;
    }

    // Iterate through window to calculate average.
    Eigen::Vector3d average;
    average.setZero();
    for(auto point = ros_node::m_gyroscope_calibration_window.cbegin(); point != ros_node::m_gyroscope_calibration_window.cend(); ++point)
    {
        average += *point;
    }
    average /= static_cast<double>(ros_node::m_gyroscope_calibration_window.size());

    // Clear window.
    ros_node::m_gyroscope_calibration_window.clear();

    // Create new homogeneous transform by subtracting out bias.
    Eigen::Matrix4d calibration;
    calibration.setIdentity();
    calibration.block(0, 3, 3, 1) = -average;

    // Update gyroscope calibration.
    ros_node::m_calibration_gyroscope.update(calibration);

    // Log success.
    ROS_INFO_STREAM("gyroscope calibration completed with averaging period of " << averaging_period << " ms");

    return true;
}

// METHODS
void ros_node::deinitialize_driver()
{
    try
    {
        ros_node::m_driver->deinitialize();
        ROS_INFO_STREAM("driver successfully deinitialized");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
    }
}

// CALLBACKS
void ros_node::data_callback(driver::data data)
{
    // Create accelerometer message.
    sensor_msgs_ext::accelerometer message_accel;
    // Set accelerations (convert from g's to m/s^2)
    message_accel.x = static_cast<double>(data.accel_x) * 9.80665;
    message_accel.y = static_cast<double>(data.accel_y) * 9.80665;
    message_accel.z = static_cast<double>(data.accel_z) * 9.80665;
    // Apply calibration.
    ros_node::m_calibration_accelerometer.calibrate(message_accel.x, message_accel.y, message_accel.z);
    // Publish message.
    ros_node::m_publisher_accelerometer.publish(message_accel);



    // Create gyroscope message.
    sensor_msgs_ext::gyroscope message_gyro;
    // Set rotation rates (convert from deg/sec to rad/sec)
    message_gyro.x = static_cast<double>(data.gyro_x) * M_PI / 180.0;
    message_gyro.y = static_cast<double>(data.gyro_y) * M_PI / 180.0;
    message_gyro.z = static_cast<double>(data.gyro_z) * M_PI / 180.0;
    // If gyroscope calibration is running, add uncalibrate data to window.
    if(ros_node::f_gyroscope_calibrating)
    {
        ros_node::m_gyroscope_calibration_window.push_back({message_gyro.x, message_gyro.y, message_gyro.z});
    }
    // Apply calibration.
    ros_node::m_calibration_gyroscope.calibrate(message_gyro.x, message_gyro.y, message_gyro.z);
    // Publish message.
    ros_node::m_publisher_gyroscope.publish(message_gyro);
    

    // HOVERBLOOM
    ros::Time current_time = ros::Time::now(); //double secs =ros::Time::now().toSec();
    // h = std_msgs.msg.Header()
    // h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    sensor_msgs::Imu message_imu;
    message_imu.header.stamp = current_time;
    message_imu.linear_acceleration.x = message_accel.x;
    message_imu.linear_acceleration.y = message_accel.y ;
    message_imu.linear_acceleration.z = message_accel.z ;
    message_imu.linear_acceleration_covariance[0] = 0.0061;
    message_imu.linear_acceleration_covariance[4] = 0.0061;
    message_imu.linear_acceleration_covariance[8] = 0.0061;

    //https://www.mathworks.com/help/fusion/ug/Estimating-Orientation-Using-Inertial-Sensor-Fusion-and-MPU-9250.html
    //GyroscopeNoiseMPU9250 = 3.0462e-06; % GyroscopeNoise (variance) in units of rad/s
    // AccelerometerNoiseMPU9250 = 0.0061; % AccelerometerNoise (variance) in units of m/s^2
    //https://www.hindawi.com/journals/sp/2017/7594763/
    // which filter to choose: http://www.olliw.eu/2013/imu-data-fusing/#chapter24
    // try complementary, then maghony/madgwick.

    /// From gyro to angular_velocity wx, wy, wz
    message_imu.angular_velocity.x = message_gyro.x;
    message_imu.angular_velocity.y = message_gyro.y;
    message_imu.angular_velocity.z = message_gyro.z;
    message_imu.angular_velocity_covariance[0] = 3.0462e-06;
    message_imu.angular_velocity_covariance[4] = 3.0462e-06;
    message_imu.angular_velocity_covariance[8] = 3.0462e-06;

    /// TODO: from gyro and accel provide rpy
    // float zeroes[10] = { 0 };
    // message_imu.orientation_covariance[0] = -1.0;
    // roll = ;
    // pitch = ;
    // yaw = ;
    
    /// TODO: from rpy to quaternion 
    // message_imu.quaternion.w = ;
    // message_imu.quaternion.x = ;
    // message_imu.quaternion.y = ;
    // message_imu.quaternion.z = ;

    ros_node::imu_publisher.publish(message_imu);


    // Check if there was a magneto overflow.
    if(std::isnan(data.magneto_x) == false)
    {
        // Create magneto message.
        sensor_msgs_ext::magnetometer message_mag;
        // Fill magnetic field strengths (convert from uT to T)
        message_mag.x = static_cast<double>(data.magneto_x) * 0.000001;
        message_mag.y = static_cast<double>(data.magneto_y) * 0.000001;
        message_mag.z = static_cast<double>(data.magneto_z) * 0.000001;
        // Apply calibration.
        ros_node::m_calibration_magnetometer.calibrate(message_mag.x, message_mag.y, message_mag.z);
        // Publish message.
        ros_node::m_publisher_magnetometer.publish(message_mag);

        // HOVERBLOOM
        // ros::Time current_time = ros::Time::now();
        sensor_msgs::MagneticField msg_mag;
        msg_mag.magnetic_field.x = message_mag.x;
        msg_mag.magnetic_field.y = message_mag.y;
        msg_mag.magnetic_field.z = message_mag.z;
        ros_node::mag_publisher.publish(msg_mag);
    }

    // Create temperature message.
    sensor_msgs_ext::temperature message_temp;
    message_temp.temperature = static_cast<double>(data.temp);
    // Publish temperature message.
    ros_node::m_publisher_temperature.publish(message_temp);
}
