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
        std::vector<Eigen::Vector3f> parsePointCloud2(const sensor_msgs::msg::PointCloud2 msg) 
        {
            std::vector<Eigen::Vector3f> points;

            // Get point step (bytes per point)
            int point_step = msg->point_step;
            int num_points = msg->width * msg->height;
            
            // Find offsets for x, y, and z fields
            int x_offset = -1, y_offset = -1, z_offset = -1;
            for (const auto &field : msg->fields) {
                if (field.name == "x") x_offset = field.offset;
                else if (field.name == "y") y_offset = field.offset;
                else if (field.name == "z") z_offset = field.offset;
            }

            // Parse point cloud data
            points.reserve(num_points);
            for (int i = 0; i < num_points; i++) {
                int data_index = i * point_step;
                const float* x_ptr = reinterpret_cast<const float*>(&msg->data[data_index + x_offset]);
                const float* y_ptr = reinterpret_cast<const float*>(&msg->data[data_index + y_offset]);
                const float* z_ptr = reinterpret_cast<const float*>(&msg->data[data_index + z_offset]);

                points.emplace_back(*x_ptr, *y_ptr, *z_ptr);
            }

            return points;
        }
        void pcd_callback(const sensor_msgs::msg::PointCloud2 msg)
        {
            std::vector<Eigen::Vector3f> points = parsePointCloud2(msg);
            for (const auto& point : points) {
                std::cout << "X: " << point.x() << ", Y: " << point.y() << ", Z: " << point.z() << std::endl;
            }
        }

        InitializeOccupancyGrid() : Node("initialize_occupancy_grid")
        {
            pcd_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_pcd",1,pcd_callback);
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