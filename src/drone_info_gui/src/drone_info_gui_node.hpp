#ifndef DRONE_INFO_GUI_NODE_HPP
#define DRONE_INFO_GUI_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tello_msgs/msg/flight_data.hpp>
#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>

class DroneInfoGuiNode : public rclcpp::Node
{
public:
    DroneInfoGuiNode();
    void flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg);

private:
    rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr subscription_;

    // Qt Widgets
    QWidget *window;
    QLabel *pitch_label;
    QLabel *roll_label;
    QLabel *yaw_label;
    QLabel *height_label;
    QLabel *battery_label;
};

#endif
