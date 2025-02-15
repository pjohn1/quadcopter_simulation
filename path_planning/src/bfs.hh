#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <memory>
#include <list>

class BFS
{
    private:
        const Eigen::MatrixXd points;
        const Eigen::Matrix<double,1,3> initial_pose;
        const Eigen::Matrix<double,1,3> goal_pose;
        const double resolution;
        std::list<PathNode> node_objects;

    public:

        BFS (const Eigen::RowVector3d initial, const Eigen::RowVector3d goal, Eigen::MatrixXd pts, double res)
         :  points(pts),initial_pose(initial),goal_pose(goal), resolution(res)
        {}

        std::vector<PathNode> search()
        {
            std::set<PathNode, PathNodeComparator> seen;
            std::list<std::shared_ptr<PathNode>> q = {std::make_shared<PathNode>(initial_pose,0.0)};
            std::vector<PathNode> path;
            bool goal_reached = false;

            while(!q.empty())
            {
                std::shared_ptr<PathNode> curr_node = *q.begin();
                q.pop_front();

                std::set<Eigen::RowVector3d, RowVector3dComparator> neighbors = get_neighbors(curr_node->pose,points,resolution,initial_pose);        
                for (Eigen::RowVector3d neighbor : neighbors) //neighbor is a row vector
                {
                    std::cout<<"node: "<<curr_node->pose<<std::endl;
                
                    double dist_from_start = (neighbor-initial_pose).norm();
                    std::shared_ptr<PathNode> n = std::make_shared<PathNode>(neighbor,curr_node,dist_from_start);
                    node_objects.push_back(*n);

                    double dist = (goal_pose-neighbor).norm();
                    if ( dist < 1e-6)
                    {
                        path = get_path(*n);
                        goal_reached=true;
                        break;
                    }

                    if (seen.find(*n) == seen.end()) //if n not in seen
                    {
                        seen.insert(*n);
                        q.push_back(n);
                    }

                }
                if (goal_reached) break;
            }
            return path;
        }
    ~BFS()
    {
        for (auto node : node_objects) delete &node;
    }

};