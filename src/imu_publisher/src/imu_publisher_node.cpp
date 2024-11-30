#include "imu_publisher_node.hpp"
#include <cmath>

ImuPublisherNode::ImuPublisherNode() : Node("imu_publisher_node")
{
    flight_data_sub_ = this->create_subscription<tello_msgs::msg::FlightData>(
        "/flight_data", 10, std::bind(&ImuPublisherNode::flight_data_callback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    position_x, position_y, position_z = 0.0;

    last_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(),"Imu node started \n");
}

void ImuPublisherNode::flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu";

    // Convert flight data (roll, pitch, yaw) to radians
    double roll_rad = msg->roll * M_PI / 180.0;
    double pitch_rad = msg->pitch * M_PI / 180.0;
    double yaw_rad = msg->yaw * M_PI / 180.0;

    /* ------- Calculate quaternion --------- */

    // yaw
    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    // pitch
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    // roll
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

    compute_position(imu_msg)
    imu_pub_->publish(imu_msg);
    RCLCPP_INFO(this->get_logger(),"Imu msg sent \n");
}

void ImuPublisherNode::compute_position(sensor_msgs::msg::Imu imu_msg){
    /* Computation of new position given linear accelerations on x,y and z axis*/

    new_time = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration<double>(new_time - last_time).count();
    last_time = new_time;

    // v[i+1] = v[i] + ax[i]*dt
    velocity_x += imu_msg.linear_acceleration.x * dt;
    velocity_y += imu_msg.linear_acceleration.y * dt;
    velocity_z += imu_msg.linear_acceleration.z * dt;

    // x[i+1] = x[i] + vx[i]*dt
    position_x += velocity_x * dt;
    position_y += velocity_y * dt;
    position_z += velocity_z * dt;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = imu_msg.header.stamp;
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = position_x;
    odom_msg.pose.pose.position.y = position_y;
    odom_msg.pose.pose.position.z = position_z;
    odom_msg.pose.pose.orientation = imu_msg.orientation;

    odom_msg.twist.twist.linear.x = velocity_x;
    odom_msg.twist.twist.linear.y = velocity_y;
    odom_msg.twist.twist.linear.z = velocity_z;

    odometry_pub_->publish(odom_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
