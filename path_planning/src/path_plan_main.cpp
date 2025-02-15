#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "bfs.hh"

class PathPlanner : public rclcpp::Node
{
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub; //vis
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub;

        BFS *bfs;
        Eigen::MatrixXd points;
        bool initialized = false;

        double x,y,z; //current_position

    public:

        PathPlanner() : Node("path_planner")
        {

            auto pose_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
            {
                x = msg.data[0];y = msg.data[1];z=msg.data[2];
            };

            auto goal_callback = [this](const geometry_msgs::msg::PoseStamped &msg) -> void
            {
                if (initialized)
                {
                    Eigen::Matrix<double,1,3> start(x,y,z);
                    Eigen::Matrix<double,1,3> goal(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
                    Eigen::Matrix<double,1,3> original_goal = goal;
                    int rows = points.rows();
                    Eigen::MatrixXd point_dist = (points.rowwise() - goal).rowwise().norm();
                    int min_row,min_col;
                    double min_value = point_dist.minCoeff(&min_row,&min_col);
                    // find closest grid point to where the goal pose is set
                    goal = points.row(min_row);
                    std::cout<<"goal: "<<goal<<std::endl;


                    double dist_value = 0.5; // all neighbors are at a max sqrt(3)*res distance away

                    bfs = new BFS(start,goal,points,dist_value);
                    std::cout<<"Finding path. . ."<<std::endl;
                    std::vector<PathNode> path = bfs->search();
                    path.push_back(PathNode(original_goal,std::make_shared<PathNode>(path.back()),1e20));
                    //add original goal so we end up there
                    std::cout<<"Got path!"<<std::endl;

                    visualization_msgs::msg::MarkerArray markers;
                    geometry_msgs::msg::PoseArray poses;
                    int c = 0;
                    for(auto &n : path)
                    {
                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = "map";
                        marker.header.stamp = this->get_clock()->now();
                        marker.id = c;
                        marker.type = visualization_msgs::msg::Marker::SPHERE;
                        marker.action = visualization_msgs::msg::Marker::ADD;

                        marker.pose.position.x = n.pose[0];
                        marker.pose.position.y = n.pose[1];
                        marker.pose.position.z = n.pose[2];

                        marker.scale.x = 0.1;
                        marker.scale.y = 0.1;
                        marker.scale.z = 0.1;

                        marker.color.r = 0.5;
                        marker.color.g = 0.0;
                        marker.color.b = 0.5;
                        marker.color.a = 1.0;

                        marker.lifetime = rclcpp::Duration::from_seconds(0);

                        markers.markers.push_back(marker);

                        geometry_msgs::msg::Pose pose;
                        pose.position.x = n.pose[0];
                        pose.position.y = n.pose[1];
                        pose.position.z = n.pose[2];

                        pose.orientation.w = 1.0;

                        poses.poses.push_back(pose);
                        c++;
                    }

                    path_pub->publish(poses);
                    marker_pub->publish(markers);
                }
            };

            goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",2,goal_callback);
            marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path",1);
            path_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/path_points",1);
            pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/quad_pose",2,pose_callback);

            std::ifstream file("/mnt/c/Desktop/quadcopter_simulation/path_planning/src/grid.txt");
            std::cout<<"Got grid"<<std::endl;
            double x,y,z;
            int current_row = 0;
            points = Eigen::MatrixXd(19955,3);
            if (file.is_open()) {
                while(file >> x >> y >> z)
                {
                    points(current_row,0) = x;
                    points(current_row,1) = y;
                    points(current_row,2) = z;

                    current_row++;
                }
                file.close();
            }
            else {
                std::cout<<"File not opening :("<<std::endl;
            }
            std::cout<<"Done parsing grid!"<<std::endl;
            initialized = true;

        }
        ~PathPlanner(){
            delete bfs;
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto visualize_node = std::make_shared<PathPlanner>();
    rclcpp::spin(visualize_node);
    rclcpp::shutdown();
    return 0;
}