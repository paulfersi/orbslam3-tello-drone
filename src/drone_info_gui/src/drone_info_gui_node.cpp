#include "drone_info_gui_node.hpp"

DroneInfoGuiNode::DroneInfoGuiNode()
    : Node("drone_info_gui_node")
{
    subscription_ = this->create_subscription<tello_msgs::msg::FlightData>(
        "/flight_data", 10, std::bind(&DroneInfoGuiNode::flight_data_callback, this, std::placeholders::_1));

    window = new QWidget;
    window->setWindowTitle("Drone Flight Info");

    pitch_label = new QLabel("Pitch: 0°");
    roll_label = new QLabel("Roll: 0°");
    yaw_label = new QLabel("Yaw: 0°");
    height_label = new QLabel("Height: 0 cm");
    battery_label = new QLabel("Battery: 0%");

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(pitch_label);
    layout->addWidget(roll_label);
    layout->addWidget(yaw_label);
    layout->addWidget(height_label);
    layout->addWidget(battery_label);
    window->setLayout(layout);

    window->show();

    // Set up a timer to spin ROS2 periodically
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [&]()
            { rclcpp::spin_some(this->shared_from_this()); });
    timer->start(16); // Approximately 60 Hz
}

void DroneInfoGuiNode::flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
{
    //Qt's event system to safely update GUI elements
    QMetaObject::invokeMethod(this, [=]()
                              {
        pitch_label->setText("Pitch: " + QString::number(msg->pitch) + "°");
        roll_label->setText("Roll: " + QString::number(msg->roll) + "°");
        yaw_label->setText("Yaw: " + QString::number(msg->yaw) + "°");
        height_label->setText("Height: " + QString::number(msg->h) + " cm");
        battery_label->setText("Battery: " + QString::number(msg->bat) + "%"); });
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication q_application(argc, argv);
    auto node = std::make_shared<DroneInfoGuiNode>();

    return q_application.exec();
}
