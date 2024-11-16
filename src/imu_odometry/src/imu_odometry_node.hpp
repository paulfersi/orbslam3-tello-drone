#ifndef IMU_ODOMETRY_NODE_HPP
#define IMU_ODOMETRY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuOdometryNode : public rclcpp::Node
{
public:
    ImuOdometryNode();

private:
    void flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg);

    rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr flight_data_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

#endif // IMU_ODOMETRY_NODE_HPP
