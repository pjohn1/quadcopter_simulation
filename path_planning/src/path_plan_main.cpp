#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "bfs.hh"

class PathPlanner : public rclcpp::Node
{
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
        BFS *bfs;

    public:

        PathPlanner() : Node("path_planner")
        {
            std::ifstream file("/mnt/c/Desktop/quadcopter_simulation/path_planning/src/grid.txt");
            std::cout<<"Got grid"<<std::endl;
            double x,y,z;
            Eigen::MatrixXd points(69564,3);
            int current_row = 0;
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

            Eigen::Matrix<double,1,3> start(0.0,0.0,0.0);
            Eigen::Matrix<double,1,3> goal(0.0,0.0,2.0);

            double dist_value = 0.5 * sqrt(3); // all neighbors are at a max sqrt(3)*res distance away

            bfs = new BFS(start,goal,points,dist_value);
            std::cout<<"Finding path. . ."<<std::endl;
            std::vector<PathNode> path = bfs->search();
            std::cout<<"Got path!"<<std::endl;
            for( auto &n : path) std::cout<<n.pose<<std::endl;

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