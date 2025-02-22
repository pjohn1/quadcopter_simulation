#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

class FreeSpaceVisualizer : public rclcpp::Node
{
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

    public:

        FreeSpaceVisualizer() : Node("free_space_visualizer")
        {
            marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/free_space",1);
            std::ifstream file("/mnt/c/Desktop/quadcopter_simulation/path_planning/src/grid.txt");
            std::cout<<"Got grid"<<std::endl;
            // Eigen::Matrix3d matrix(81026,3);
            std::vector<geometry_msgs::msg::Point> v;
            double x,y,z;
            int count=0;
            if (file.is_open()) {
                while(file >> x >> y >> z)
                {
                    geometry_msgs::msg::Point p = geometry_msgs::msg::Point();
                    p.x = x;p.y=y;p.z=z;
                    if (count > 100 && count%100==0) {v.push_back(p);}
                    count++;
                }
                file.close();
            }
            else {
                std::cout<<"File not opening :("<<std::endl;
            }

            std::cout<<"Done parsing file!"<<std::endl;

            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "points";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05; // Point size
            marker.scale.y = 0.05;
            marker.color.a = 1.0;  // Fully visible
            marker.color.r = 1.0;  // Red color

            marker.points = v;
            marker_array.markers.push_back(marker);
            printf("Publishing %i points :)\n",static_cast<int>(v.size()));
            marker_pub->publish(marker_array);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto visualize_node = std::make_shared<FreeSpaceVisualizer>();
    rclcpp::spin(visualize_node);
    rclcpp::shutdown();
    return 0;
}