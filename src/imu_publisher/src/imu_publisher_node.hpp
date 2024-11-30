#ifndef IMU_PUBLISHER_NODE_HPP
#define IMU_PUBLISHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include <chrono>


class ImuPublisherNode : public rclcpp::Node
{
public:
    ImuPublisherNode();

private:
    void flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg);
    void compute_position(sensor_msgs::msg::Imu imu_msg);
    void publish_tf(nav_msgs::msg::Odometry odom_msg);

    rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr flight_data_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double position_x,position_y, position_z;
    double velocity_x, velocity_y,velocity_z;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time, new_time;

};
#endif // IMU_PUBLISHER_NODE_HPP
