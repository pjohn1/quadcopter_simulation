#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <memory>
#include <queue>
#include "search_functions.hh"
#include <list>

class AStar
{
    private:
        const Eigen::MatrixXd points;
        const Eigen::Matrix<double,1,3> initial_pose;
        const Eigen::Matrix<double,1,3> goal_pose;
        const double resolution;
        std::list<PathNode> node_objects;

    public:

        AStar (const Eigen::RowVector3d initial, const Eigen::RowVector3d goal, Eigen::MatrixXd pts, double res)
         :  points(pts),initial_pose(initial),goal_pose(goal), resolution(res)
        {}

        struct Comparator
        {
            bool operator()(const std::shared_ptr<PathNode>& p1, const std::shared_ptr<PathNode>& p2) const {
                if (p1->f_score != p2->f_score) return p1->f_score > p2->f_score;
                return false;
            }
        };


        std::vector<PathNode> search()
        {
            std::set<PathNode, PathNodeComparator> seen;
            std::priority_queue<std::shared_ptr<PathNode>,std::vector<std::shared_ptr<PathNode>>,AStar::Comparator> q;
            q.push(std::make_shared<PathNode>(initial_pose,0.0));
            std::vector<PathNode> path;
            bool goal_reached = false;

            while(!q.empty())
            {
                std::shared_ptr<PathNode> curr_node = q.top();
                // std::cout<<"current: "<<curr_node->f_score<<std::endl;
                q.pop();

                std::set<Eigen::RowVector3d, RowVector3dComparator> neighbors = get_neighbors(curr_node->pose,points,resolution,initial_pose);        
                for (Eigen::RowVector3d neighbor : neighbors) //neighbor is a row vector
                {
                    // std::cout<<"node: "<<neighbor<<std::endl;
                
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