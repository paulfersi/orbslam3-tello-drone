#include "imu_odometry_node.hpp"
#include <cmath>

ImuOdometryNode::ImuOdometryNode()
    : Node("imu_odometry_node"), x_(0.0), y_(0.0), theta_(0.0), vx_(0.0), vy_(0.0)
{
    flight_data_sub_ = this->create_subscription<tello_msgs::msg::FlightData>(
        "/flight_data", 10, std::bind(&ImuOdometryNode::flight_data_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
}

void ImuOdometryNode::flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "base_link";

    // Convert flight data (roll, pitch, yaw) to radians
    double roll_rad = msg->roll * M_PI / 180.0;
    double pitch_rad = msg->pitch * M_PI / 180.0;
    double yaw_rad = msg->yaw * M_PI / 180.0;

    // Calculate quaternion from roll, pitch, and yaw
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

    // Assume time delta between messages (in seconds)
    double dt = 0.1; // Adjust this based on actual message frequency

    // Calculate angular velocity change (yaw rate only, based on z-axis)
    double dtheta = imu_msg.angular_velocity.z * dt;
    theta_ += dtheta;

    // Remove gravity effect from acceleration data
    double accel_x_world = imu_msg.linear_acceleration.x * cos(theta_) - imu_msg.linear_acceleration.y * sin(theta_);
    double accel_y_world = imu_msg.linear_acceleration.x * sin(theta_) + imu_msg.linear_acceleration.y * cos(theta_);

    // Integrate acceleration to get velocity
    vx_ += accel_x_world * dt;
    vy_ += accel_y_world * dt;

    // Integrate velocity to get position
    x_ += vx_ * dt;
    y_ += vy_ * dt;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation = imu_msg.orientation;

    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = imu_msg.angular_velocity.z;

    odom_pub_->publish(odom_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
