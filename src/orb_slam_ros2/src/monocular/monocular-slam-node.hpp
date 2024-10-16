#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <string> // Include string header

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/msg/compressed_image.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System *pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System *m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr quaternion_pub;

    // Define frame IDs
    std::string header_id_frame = "camera_frame"; // Replace with your desired frame ID
    std::string child_id_frame = "odom_frame";    // Replace with your desired child frame ID

    // Add other member variables if necessary
    double scale_position_mono;   // Ensure these are defined and initialized
    double degree_move_pose_mono; // Ensure these are defined and initialized
};

#endif
