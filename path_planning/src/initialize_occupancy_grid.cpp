#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <Eigen/Dense>

class InitializeOccupancyGrid : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub;
    public:
        InitializeOccupancyGrid() : Node("initialize_occupancy_grid")
        {
            // pcd_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_pcd",1,pcd_callback);
            pcd_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in",1);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto map_initializer = std::make_shared<InitializeOccupancyGrid>();
    rclcpp::spin(map_initializer);
    rclcpp::shutdown();
    return 0;
}