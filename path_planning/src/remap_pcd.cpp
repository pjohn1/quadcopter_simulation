#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

class PcdConverter : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub;

    public:

        PcdConverter() : Node("PcdConverter")
        {
            auto pcd_callback = [this](const sensor_msgs::msg::PointCloud2 msg) -> void
            {
                sensor_msgs::msg::PointCloud2 new_msg = sensor_msgs::msg::PointCloud2();
                new_msg = msg;
                new_msg.header.frame_id = "map";
                pcd_pub->publish(new_msg);
            };
            pcd_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_pcd",1,pcd_callback);
            pcd_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in",1);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto pcd_node = std::make_shared<PcdConverter>();
    rclcpp::spin(pcd_node);
    rclcpp::shutdown();
    return 0;
}