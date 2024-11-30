#include "imu_publisher_node.hpp"
#include <cmath>

ImuPublisherNode::ImuPublisherNode() : Node("imu_publisher_node")
{
    flight_data_sub_ = this->create_subscription<tello_msgs::msg::FlightData>(
        "/flight_data", 10, std::bind(&ImuPublisherNode::flight_data_callback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    
    position_x, position_y, position_z = 0.0;

    last_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(),"Imu node started \n");
}

void ImuPublisherNode::flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "orbslam3";

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

    imu_pub_->publish(imu_msg);
    RCLCPP_INFO(this->get_logger(),"Imu msg sent \n");
}

void ImuPublisherNode::compute_position(sensor_msgs::msg::Imu imu_msg){
    /* Computation of new position given linear accelerations on x,y and z axis*/

    new_time = std::chrono::high_resolution_clock::now();

    dt = (new_time - last_time).count();         //converts duration to double

    

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
