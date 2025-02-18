#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <list>

#define MIN_DIST 0.5

class PathPointPublisher : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub; //vis
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr points_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub;

        std::list<Eigen::RowVector3d> points;
        bool initialized = false;

        Eigen::RowVector3d current_point;

    public:

        PathPointPublisher() : Node("path_point_publisher")
        {
            auto points_callback = [this](const geometry_msgs::msg::PoseArray &msg) -> void {

                for (auto &point : msg.poses)
                {
                    Eigen::RowVector3d p(point.position.x,point.position.y,point.position.z);

                    points.push_back(p);
                }
                current_point = points.front();
                points.pop_front();
                initialized = true;
            };

            auto pose_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
            {
                if (initialized)
                {
                    Eigen::RowVector3d pose(msg.data[0],msg.data[1],msg.data[2]);
                    geometry_msgs::msg::PointStamped point;
                    point.header.frame_id = "map";
                    point.header.stamp = this->get_clock()->now();

                    double dist = (current_point - pose).norm();

                    if (!points.empty() && dist < MIN_DIST)
                    {
                        current_point = points.front();
                        points.pop_front();
                    }

                    point.point.x = current_point[0];
                    point.point.y = current_point[1];
                    point.point.z = current_point[2];

                    point_pub->publish(point);
                }
            };

            pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/quad_pose",2,pose_callback);
            points_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/path_points",2,points_callback);
            point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/current_point",2);

        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto point_publisher = std::make_shared<PathPointPublisher>();
    rclcpp::spin(point_publisher);
    rclcpp::shutdown();
    return 0;
}