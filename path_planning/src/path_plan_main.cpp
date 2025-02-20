#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "astar.hh"
#include "polyfit.hh"

#define NUM_POINTS 1500235
#define RES 1.0
#define HEIGHT_ABOVE 2.0

class PathPlanner : public rclcpp::Node
{
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub; //vis
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub;

        // BFS *bfs;
        AStar *astar;
        Eigen::MatrixXd points;
        bool initialized = false;
        Eigen::Matrix<double,1,3> goal;

        bool pose_intialized = false;

        KDTreeEigenMatrixAdaptor<double> *kdtree;

        double x,y,z; //current_position

    public:

        Eigen::RowVector3d get_closest(Eigen::MatrixXd points, Eigen::RowVector3d loc)
        {
            int rows = points.rows();
            Eigen::MatrixXd point_dist = (points.rowwise() - loc).rowwise().norm();
            int min_row,min_col;
            double min_value = point_dist.minCoeff(&min_row,&min_col);
            // find closest grid point to where the goal pose is set
            return points.row(min_row);
        }

        std::vector<PathNode> shift_points(std::vector<PathNode>& path)
        {
            double path_length = ((path.back()).pose - path[0].pose).norm();
            Eigen::RowVector3d first_node = path[0].pose;
            for (auto &node : path)
            {
                double dist = (node.pose - first_node).norm();
                double new_z = HEIGHT_ABOVE*std::sin(M_PI*dist/path_length);
                node.pose[2] += new_z;
            }
            return path;
            
        }

        PathPlanner() : Node("path_planner")
        {

            auto pose_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
            {
                x = msg.data[0];y = msg.data[1];z=msg.data[2];

                if (!pose_intialized) //only do this on first start to ensure x,y,z are set
                {
                    std::ifstream file("/mnt/c/Desktop/quadcopter_simulation/path_planning/src/grid.txt");
                    std::cout<<"Got grid"<<std::endl;
                    double xf,yf,zf;
                    int current_row = 0;
                    points = Eigen::MatrixXd(NUM_POINTS,3);
                    if (file.is_open()) {
                        while(file >> xf >> yf >> zf)
                        {
                            points(current_row,0) = xf;
                            points(current_row,1) = yf;
                            points(current_row,2) = zf;
        
                            current_row++;
                        }
                        file.close();
                    }
                    else {
                        std::cout<<"File not opening :("<<std::endl;
                    }
                    std::cout<<"Done parsing grid!"<<std::endl;
                    kdtree = new KDTreeEigenMatrixAdaptor<double>(points);
                    std::cout<<"Created KDTree!"<<std::endl;
                    initialized = true;
                    pose_intialized = true;
                }
            };

            auto goal_callback = [this](const geometry_msgs::msg::PointStamped &msg) -> void
            {
                if (initialized && goal[0] != msg.point.x && goal[1] != msg.point.y && goal[2] != msg.point.z)
                {
                    Eigen::Matrix<double,1,3> start(x,y,z);
                    std::cout<<"start: "<<start<<std::endl;
                    goal << msg.point.x,msg.point.y,msg.point.z;
                    Eigen::Matrix<double,1,3> original_goal = goal;
                    goal = get_closest(points,goal);
                    std::cout<<"goal: "<<goal<<std::endl;


                    double dist_value = RES * sqrt(3.0); // all neighbors are at a max sqrt(3)*res distance away

                    // bfs = new BFS(start,goal,points,dist_value);
                    astar = new AStar(start,goal,points,dist_value,*kdtree);
                    std::cout<<"Finding path. . ."<<std::endl;
                    double t1 = this->get_clock()->now().seconds();
                    std::vector<PathNode> path = astar->search();
                    if (!path.empty())
                    {
                        std::cout<<"Found path in (s): "<<this->get_clock()->now().seconds() - t1<<std::endl;
                        path.push_back(PathNode(original_goal,std::make_shared<PathNode>(path.back()),1e20));
                        //add original goal so we end up there
                        // std::cout<<"Got path! Polyfitting..."<<std::endl;
                        // int degree = 1;
                        // double rsquared = 0;
                        // while (rsquared < 0.8 && degree<6)
                        // {
                        //     Eigen::VectorXd coeffs = polyfit2D(path,degree);
                        //     std::cout<<"coeffs: "<<coeffs<<std::endl;
                        //     double rsquared = computeFitQuality(coeffs,path,degree);
                        //     std::cout<<"degree: "<<degree<<" rsqured: "<<rsquared<<std::endl;
                        //     degree++;
                        // }
                        path = shift_points(path);


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
                }
                else {
                    std::cout<<"No path found :("<<std::endl;
                }
            };

            pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/quad_pose",2,pose_callback);
            goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point",2,goal_callback);
            marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path",1);
            path_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/path_points",1);


        }
        ~PathPlanner(){
            delete astar;
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