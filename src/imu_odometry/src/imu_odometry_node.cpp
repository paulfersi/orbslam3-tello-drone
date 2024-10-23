#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0), vx_(0.0), vy_(0.0), vtheta_(0.0)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&OdometryNode::imu_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Assume time delta between messages (in seconds)
        double dt = 0.1; // Replace this with actual delta time between IMU messages

        // Convert angular velocity (from IMU) to change in orientation (theta)
        double dtheta = msg->angular_velocity.z * dt;
        theta_ += dtheta;

        // Linear acceleration (from IMU)
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;

        // Remove gravity from accelerometer data (assuming orientation is known)
        // This part will need orientation estimation from the IMU quaternion
        double accel_x_world = ax * cos(theta_) - ay * sin(theta_);
        double accel_y_world = ax * sin(theta_) + ay * cos(theta_);

        // Integrate acceleration to get velocity
        vx_ += accel_x_world * dt;
        vy_ += accel_y_world * dt;

        // Integrate velocity to get position
        x_ += vx_ * dt;
        y_ += vy_ * dt;

        // Publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // Orientation (from IMU)
        odom_msg.pose.pose.orientation = msg->orientation;

        // Velocity
        odom_msg.twist.twist.linear.x = vx_;
        odom_msg.twist.twist.linear.y = vy_;
        odom_msg.twist.twist.angular.z = msg->angular_velocity.z;

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double x_, y_, theta_;    // Position and orientation
    double vx_, vy_, vtheta_; // Velocities
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
