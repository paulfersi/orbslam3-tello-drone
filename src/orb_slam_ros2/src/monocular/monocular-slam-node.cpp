#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>("topic_pub_quat", 10);
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

//callback

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg, "bgr8");  //for compressed imgs
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;

    //call orbslam
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    // Obtain the position and the orientation
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    // I publish position and quaternion (rotated)
    auto message = nav_msgs::msg::Odometry();
    geometry_msgs::msg::Pose output_pose{};

    // Multipling is necessary in the monocular, since there is no depth info
    output_pose.position.x = twc.z() * this->scale_position_mono;
    output_pose.position.y = -twc.x() * this->scale_position_mono;
    output_pose.position.z = 0;

    output_pose.orientation.x = -q.z();
    output_pose.orientation.y = -q.x();
    output_pose.orientation.z = -q.y();
    output_pose.orientation.w = q.w();

    if (this->degree_move_pose_mono != 0)
    {
        // If required, I move the yaw of specified degrees
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(output_pose.orientation, tf2_quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(tf2_quat);
        m.getRPY(roll, pitch, yaw);

        tf2_quat.setRPY(0, 0, yaw + (this->degree_move_pose_mono * (M_PI / 180.0)));
        output_pose.orientation = tf2::toMsg(tf2_quat);
    }

    message.pose.pose = output_pose;
    message.header.frame_id = header_id_frame;
    message.child_frame_id = child_id_frame;

    // add timestamp to message
    message.header.stamp = this->now();

    quaternion_pub->publish(message);
}
