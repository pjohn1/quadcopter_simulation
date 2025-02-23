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
        std::vector<std::shared_ptr<PathNode>> points;
        const Eigen::Matrix<double,1,3> initial_pose;
        const Eigen::Matrix<double,1,3> goal_pose;

        const double resolution;

        KDTreePathNodeAdaptor<double> kdtree;
        std::shared_ptr<rclcpp::Node> pub_node;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        int marker_id = 0;

    public:

        AStar (const Eigen::RowVector3d initial, const Eigen::RowVector3d goal, std::vector<std::shared_ptr<PathNode>> pts, 
            double res, KDTreePathNodeAdaptor<double>& tree)
         :  points(pts),initial_pose(initial),goal_pose(goal), resolution(res), kdtree(tree)
        {
            pub_node = rclcpp::Node::make_shared("vis_node");
            marker_pub = pub_node->create_publisher<visualization_msgs::msg::Marker>("/marker_vis",1);
        }

        struct Comparator
        {
            //used to sort the priority queue, sort by f_score (cost to node + h(node,goal))
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

        std::vector<PathNode> search(std::shared_ptr<PathNode> initial_node)
        {
            std::set<PathNode, PathNodeComparator> seen;
            std::priority_queue<std::shared_ptr<PathNode>,std::vector<std::shared_ptr<PathNode>>,AStar::Comparator> q;
            q.push(initial_node); //initialize q w initial pose
            std::vector<PathNode> path;
            bool goal_reached = false;

            while(!q.empty())
            {
                std::shared_ptr<PathNode> curr_node = q.top();
                // publish_marker(curr_node->pose);
                // std::cout<<"current: "<<curr_node->pose<<" fscore: "<<curr_node->f_score<<std::endl;
                q.pop();

                //get top node and check neighbors

                std::vector<std::shared_ptr<PathNode>> neighbors = get_neighbors_butbetter(kdtree,curr_node->pose,resolution,initial_pose);        
                for (std::shared_ptr<PathNode> neighbor : neighbors)
                {
                    Eigen::RowVector3d neighbor_pose = neighbor->pose;
                    double dist_from_goal = (goal_pose-neighbor_pose).norm();
                    double dist_from_neighbor = (neighbor_pose-curr_node->pose).norm();
                    double tentative_gscore = curr_node->g_score + dist_from_neighbor;

                    if ( dist_from_goal < 1e-6)
                    {
                        std::cout<<"Goal found!"<<std::endl;
                        neighbor->has_parent = true;
                        neighbor->parent = curr_node;
                        path = get_path(*neighbor);
                        std::cout<<"Got Path!"<<std::endl;
                        goal_reached=true;
                        break;
                    }

                    if (tentative_gscore <= neighbor->g_score && seen.find(*neighbor) == seen.end())
                    {
                        //if cost from current node + h(node, neighbor)
                        //is less than cost from start node to neighbor node,
                        //add to q to explore neighbors

                        neighbor->parent = curr_node;
                        neighbor->has_parent = true;
                        neighbor->g_score = tentative_gscore;
                        neighbor->f_score = neighbor->g_score+dist_from_goal;
                        //update neighbor node directly 
                        seen.insert(*neighbor);

                        q.push(neighbor);
                    }

                }
                if (goal_reached) break;
            }
            return path;
        }
    ~AStar()
    {}

};