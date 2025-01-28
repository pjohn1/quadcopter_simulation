#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include "drone_properties.hpp"

class VelocityConverter : rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray> velocity_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray> voltage_publisher;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto controller_node = std::make_shared<VelocityConverter>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}