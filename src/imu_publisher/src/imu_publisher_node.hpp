#ifndef IMU_PUBLISHER_NODE_HPP
#define IMU_PUBLISHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>


class ImuPublisherNode : public rclcpp::Node
{
public:
    ImuPublisherNode();

private:
    void flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg);

    rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr flight_data_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    double position_x, position_y, position_z;

    auto last_time, new_time;

    double dt;

#endif // IMU_PUBLISHER_NODE_HPP
