#ifndef __MONOCULAR_INERTIAL_HPP__
#define __MONOCULAR_INERTIAL_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <deque>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "orbslam3_odometry/utility.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h> // Include for the TransformBroadcaster

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class MonoInertialSlamNode : public rclcpp::Node
{
public:
    MonoInertialSlamNode(ORB_SLAM3::System *pSLAM);
    ~MonoInertialSlamNode();

private:

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void GrabIMUData(const sensor_msgs::msg::Imu::SharedPtr msg);

    void ProcessSlam();

    void loadParameters();

    std::string camera_topic, imu_topic, header_id_frame, child_id_frame, topic_pub_quat;
    float scale_position_mono;
    float degree_move_pose_mono;

    int cutting_x, cutting_y, cutting_width, cutting_height;
    cv::Rect cutting_rect;

    // ORB-SLAM3 system
    ORB_SLAM3::System *m_SLAM;

    // Image and IMU buffers for synchronization
    std::deque<std::pair<cv::Mat, rclcpp::Time>> m_image_buffer;
    std::deque<ORB_SLAM3::IMU::Point> m_imu_buffer;

    // Mutex for buffer synchronization
    std::mutex m_buffer_mutex;


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_subscriber;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr quaternion_pub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherPointCloud;

    // Transform broadcaster for publishing transformations
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // __MONOCULAR_INERTIAL_HPP__
