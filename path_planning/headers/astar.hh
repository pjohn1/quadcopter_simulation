#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <memory>
#include <queue>
#include "search_functions.hh"
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class AStar
{
    private:
        Eigen::MatrixXd points;
        const Eigen::Matrix<double,1,3> initial_pose;
        const Eigen::Matrix<double,1,3> goal_pose;
        const double resolution;
        std::list<PathNode> node_objects;
        KDTreeEigenMatrixAdaptor<double> kdtree;
        std::shared_ptr<rclcpp::Node> pub_node;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        int marker_id = 0;

    public:

        AStar (const Eigen::RowVector3d initial, const Eigen::RowVector3d goal, Eigen::MatrixXd pts, 
            double res, KDTreeEigenMatrixAdaptor<double>& tree)
         :  points(pts),initial_pose(initial),goal_pose(goal), resolution(res), kdtree(tree)
        {
            pub_node = rclcpp::Node::make_shared("vis_node");
            marker_pub = pub_node->create_publisher<visualization_msgs::msg::Marker>("/marker_vis",1);
        }

        struct Comparator
        {
            bool operator()(const std::shared_ptr<PathNode>& p1, const std::shared_ptr<PathNode>& p2) const {
                if (p1->f_score != p2->f_score) return p1->f_score > p2->f_score;
                return false;
            }
        };

        void publish_marker(Eigen::RowVector3d point)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = pub_node->get_clock()->now();
            marker.id = marker_id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = point[0];
            marker.pose.position.y = point[1];
            marker.pose.position.z = point[2];

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.color.r = 0.9;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration::from_seconds(0);

            marker_pub->publish(marker);
        }

        std::vector<PathNode> search()
        {
            std::set<PathNode, PathNodeComparator> seen;
            std::priority_queue<std::shared_ptr<PathNode>,std::vector<std::shared_ptr<PathNode>>,AStar::Comparator> q;
            q.push(std::make_shared<PathNode>(initial_pose,0.0));
            std::vector<PathNode> path;
            bool goal_reached = false;

            // GetNeighbors *neighbor_struct = new GetNeighbors(&points);

            while(!q.empty())
            {
                std::shared_ptr<PathNode> curr_node = q.top();
                // publish_marker(curr_node->pose);
                // std::cout<<"current: "<<curr_node->pose<<" fscore: "<<curr_node->f_score<<std::endl;
                q.pop();

                std::set<Eigen::RowVector3d, RowVector3dComparator> neighbors = get_neighbors_butbetter(kdtree,curr_node->pose,resolution,initial_pose,points);        
                for (Eigen::RowVector3d neighbor : neighbors) //neighbor is a row vector
                {
                    double dist_from_start = (neighbor-initial_pose).norm();
                    double dist_from_goal = (goal_pose-neighbor).norm();
                    double dist_from_neighbor = (neighbor-curr_node->pose).norm();
                    double tentative_gscore = curr_node->g_score + dist_from_neighbor;


                    std::shared_ptr<PathNode> n = std::make_shared<PathNode>(neighbor,curr_node,dist_from_start,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity());
                    node_objects.push_back(*n);
                    
                    // auto found_n = seen.find(*n);
                    // // see if we already have this node
                    // if (found_n != seen.end())
                    // {
                    //     n->updateNode(*found_n);
                    //     std::cout<<"eneted"<<std::endl;
                    //     seen.erase(found_n);
                    //     std::cout<<"left"<<std::endl;
                    // }

                    if ( dist_from_goal < 1e-6)
                    {
                        std::cout<<"Goal found!"<<std::endl;
                        path = get_path(*n);
                        std::cout<<"Got Path!"<<std::endl;
                        goal_reached=true;
                        break;
                    }

                    if (seen.find(*n) == seen.end() && tentative_gscore <= n->g_score)
                    {

                        seen.insert(*n);
                        n->g_score = tentative_gscore;
                        n->f_score = n->g_score+dist_from_goal;

                        q.push(n);
                    }

                }
                if (goal_reached) break;
            }
            return path;
        }
    ~AStar()
    {
        for (auto node : node_objects) delete &node;
    }

};