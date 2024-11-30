#include "monocular_inertial.hpp"
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;

void MonoInertialSlamNode::loadParameters()
{
    declare_parameter("topic_camera", "/camera/image");
    declare_parameter("topic_imu", "/imu/data");
    declare_parameter("topic_orbslam_odometry", "/Odometry/orbSlamOdom");
    declare_parameter("topic_header_frame_id", "os_track");
    declare_parameter("topic_child_frame_id", "orbslam3");
    declare_parameter("scale_position_mono", 1.0);
    declare_parameter("degree_move_pose_mono", 0.0);
    declare_parameter("cutting_x", -1);
    declare_parameter("cutting_y", 0);
    declare_parameter("cutting_width", 0);
    declare_parameter("cutting_height", 0);

    get_parameter("topic_camera", this->camera_topic);
    get_parameter("topic_imu", this->imu_topic);
    get_parameter("topic_orbslam_odometry", this->topic_pub_quat);
    get_parameter("topic_header_frame_id", this->header_id_frame);
    get_parameter("topic_child_frame_id", this->child_id_frame);
    get_parameter("scale_position_mono", this->scale_position_mono);
    get_parameter("degree_move_pose_mono", this->degree_move_pose_mono);
    get_parameter("cutting_x", this->cutting_x);
    get_parameter("cutting_y", this->cutting_y);
    get_parameter("cutting_width", this->cutting_width);
    get_parameter("cutting_height", this->cutting_height);
}

MonoInertialSlamNode::MonoInertialSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("orbslam3_mono_inertial")
{
    this->loadParameters();

    // Compute cutting rect
    if (cutting_x != -1)
        cutting_rect = cv::Rect(cutting_x, cutting_y, cutting_width, cutting_height);

    m_SLAM = pSLAM;

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        this->camera_topic,
        qos,
        std::bind(&MonoInertialSlamNode::GrabImage, this, std::placeholders::_1));

    m_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        this->imu_topic,
        qos,
        std::bind(&MonoInertialSlamNode::GrabIMUData, this, std::placeholders::_1));

    quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>(topic_pub_quat, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::Transforquaternion_pubBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 STARTED IN MONO-INERTIAL MODE.");
}

MonoInertialSlamNode::~MonoInertialSlamNode()
{
    m_SLAM->Shutdown();
    RCLCPP_INFO(this->get_logger(), "Exiting and saving trajectories...");
    m_SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
}

void MonoInertialSlamNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image_for_orbslam = m_cvImPtr->image;
    if (cutting_x != -1)
        Utility::cutting_image(image_for_orbslam, cutting_rect);

    std::lock_guard<std::mutex> lock(m_buffer_mutex);
    m_image_buffer.push_back(std::make_pair(image_for_orbslam, msg->header.stamp));
}

void MonoInertialSlamNode::GrabIMUData(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    ORB_SLAM3::IMU::Point imu_point(
        Eigen::Vector3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
        Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Utility::StampToSec(msg->header.stamp));

    std::lock_guard<std::mutex> lock(m_buffer_mutex);
    m_imu_buffer.push_back(imu_point);
}

void MonoInertialSlamNode::ProcessSlam()
{
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::lock_guard<std::mutex> lock(m_buffer_mutex);

        if (!m_image_buffer.empty() && !m_imu_buffer.empty())
        {
            auto image_pair = m_image_buffer.front();
            m_image_buffer.pop_front();

            cv::Mat image = image_pair.first;
            double timestamp = Utility::StampToSec(image_pair.second);

            m_SLAM->TrackMonocularInertial(image, timestamp, m_imu_buffer);
            m_imu_buffer.clear();
        }
    }
}
