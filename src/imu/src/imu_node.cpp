#include "imu_node.hpp"
#include <cmath>

ImuNode::ImuNode() : Node("imu_node")
{
    flight_data_sub_ = this->create_subscription<tello_msgs::msg::FlightData>(
    "/flight_data", 10, std::bind(&ImuNode::flight_data_callback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
}

void ImuNode::flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "base_link";

    double roll_rad = msg->roll * M_PI / 180.0;
    double pitch_rad = msg->pitch * M_PI / 180.0;
    double yaw_rad = msg->yaw * M_PI / 180.0;

    // Calculate quaternion
    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cr = cos(roll_rad * 0.5);
    double sr = sin(roll_rad * 0.5);

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

    // Linear acceleration
    imu_msg.linear_acceleration.x = msg->agx / 1000.0;
    imu_msg.linear_acceleration.y = msg->agy / 1000.0;
    imu_msg.linear_acceleration.z = msg->agz / 1000.0;

    // Angular velocity (assuming 0 if not available)
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;

    imu_pub_->publish(imu_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
