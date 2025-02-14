#include <Eigen/Dense>
#include <set>
#include <iostream>
#include <vector>

struct PathNode {
    Eigen::RowVector3d pose;
    std::shared_ptr<PathNode> parent;
    bool has_parent;
    PathNode(){}
    PathNode(Eigen::RowVector3d curr_pose, std::shared_ptr<PathNode> last_node)
    : pose(curr_pose), parent(last_node), has_parent(true) {}
    PathNode(Eigen::RowVector3d curr_pose, double last_node)
    : pose(curr_pose),has_parent(false) {}
};

struct RowVector3dComparator {
bool operator()(const Eigen::RowVector3d& lhs, const Eigen::RowVector3d& rhs) const {
    if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();
    if (lhs.y() != rhs.y()) return lhs.y() < rhs.y();
    return lhs.z() < rhs.z();
}
};

struct PathNodeComparator {
bool operator()(const PathNode& lhs, const PathNode& rhs) const {
    if ( lhs.pose[0] != rhs.pose[0] ) return lhs.pose[0] < rhs.pose[0];
    if ( lhs.pose[1] != rhs.pose[1]) return lhs.pose[1] < rhs.pose[1];
    return lhs.pose[2] < rhs.pose[2];
}
};

std::set<Eigen::RowVector3d, RowVector3dComparator> get_neighbors(Eigen::RowVector3d loc, Eigen::MatrixXd points,
double resolution)
{    
    Eigen::MatrixXd dist(points.rows(),points.cols());
    dist = points.rowwise() - loc;
    Eigen::MatrixXd new_dist(points.rows(),1);
    new_dist = dist.rowwise().norm();

    Eigen::MatrixXd mask = (new_dist.array() <= (resolution + 1e-2) && new_dist.array() > 1e-3).cast<double>();
    Eigen::MatrixXd masked_points(points.rows(),points.cols());
    for (int r=0;r<points.rows();r++)
    {
        Eigen::Matrix<double,1,3> row_mult = points.row(r) * mask(r,0);

        masked_points(r,0) = row_mult[0];
        masked_points(r,1) = row_mult[1];
        masked_points(r,2) = row_mult[2];
    }
    std::set<Eigen::RowVector3d, RowVector3dComparator> neighbors;

    for(int i=0;i<masked_points.rows();i++)
    {
        neighbors.insert(masked_points.row(i));
    }
    
    return neighbors;

}

std::vector<PathNode> get_path(PathNode start_node)
{
    PathNode current_node = start_node;
    std::vector<PathNode> path = {start_node};
    double max_iter = 1000;
    double iter=0;
    while (current_node.has_parent)
    {
        path.insert(path.begin(),*current_node.parent);
        current_node = *current_node.parent;
        iter++;
        if (iter > max_iter) break;
    }

    return path;
}