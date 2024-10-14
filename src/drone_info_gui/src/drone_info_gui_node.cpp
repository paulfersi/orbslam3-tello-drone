#include "drone_info_gui_node.hpp"


class DroneInfoGuiNode : public rclcpp::Node
{
public:
    DroneInfoGuiNode()
        : Node("drone_info_gui_node")
    {
        subscription_ = this->create_subscription<tello_msgs::msg::FlightData>(
            "/flight_data", 10, std::bind(&DroneInfoGuiNode::flight_data_callback, this, _1));
    }

private:
    void flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg) 
    {
        continue;
    }
    rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
