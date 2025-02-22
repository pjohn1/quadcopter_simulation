#include <Eigen/Dense>
#include <set>
#include <iostream>
#include <vector>
#include <nanoflann.hpp>

template <typename num_t>
struct KDTreeEigenMatrixAdaptor {
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<num_t, KDTreeEigenMatrixAdaptor<num_t>>,
        KDTreeEigenMatrixAdaptor<num_t>,
        3
    > KDTree;

    Eigen::MatrixXd points;
    KDTree* index;

    KDTreeEigenMatrixAdaptor(const Eigen::MatrixXd& pts)
        : points(pts) {
        index = new KDTree(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        index->buildIndex();
    }

    ~KDTreeEigenMatrixAdaptor() { delete index; }

    inline size_t kdtree_get_point_count() const { return points.rows(); }

    inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const {
        return points(idx, dim);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};


struct PathNode {
    Eigen::RowVector3d pose;
    std::shared_ptr<PathNode> parent;
    double cost;
    double g_score;
    double f_score;
    bool has_parent;

    void updateNode(const PathNode &p2)
    {
        this->pose = p2.pose;
        this->parent = p2.parent;
        this->cost = p2.cost;
        this->g_score = p2.g_score;
        this->f_score = p2.f_score;
    }

    PathNode(){}
    PathNode(Eigen::RowVector3d curr_pose, std::shared_ptr<PathNode> last_node, double dist)
    : pose(curr_pose), parent(last_node), has_parent(true), cost(dist) {}
    PathNode(Eigen::RowVector3d curr_pose, double last_node)
    : pose(curr_pose),has_parent(false),cost(0.0),g_score(0.0) {}
    PathNode(Eigen::RowVector3d curr_pose, std::shared_ptr<PathNode> last_node, double dist, double gscore, double fscore)
    : pose(curr_pose),parent(last_node),has_parent(true),cost(dist),g_score(gscore),f_score(fscore) 
    {}
};

struct RowVector3dComparator {

    Eigen::RowVector3d initial_pose;
    RowVector3dComparator(const Eigen::RowVector3d& ref) : initial_pose(ref) {}

    bool operator()(const Eigen::RowVector3d& lhs, const Eigen::RowVector3d& rhs) const {
        double l_dist = (lhs - initial_pose).norm();
        double r_dist = (rhs - initial_pose).norm();

        if (l_dist != r_dist) return l_dist < r_dist;
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
double resolution, Eigen::RowVector3d initial_pose)
{    
    Eigen::MatrixXd dist(points.rows(),points.cols());
    dist = points.rowwise() - loc;
    Eigen::MatrixXd new_dist(points.rows(),1);
    new_dist = dist.rowwise().squaredNorm();

    Eigen::MatrixXd mask = (new_dist.array() <= (pow(resolution,2) + 1e-2) && new_dist.array() > 1e-3).cast<double>();
    Eigen::MatrixXd masked_points(points.rows(),points.cols());
    for (int r=0;r<points.rows();r++)
    {
        Eigen::Matrix<double,1,3> row_mult = points.row(r) * mask(r,0);

        masked_points(r,0) = row_mult[0];
        masked_points(r,1) = row_mult[1];
        masked_points(r,2) = row_mult[2];
    }
    std::set<Eigen::RowVector3d, RowVector3dComparator> neighbors(initial_pose);

    for(int i=0;i<masked_points.rows();i++)
    {
        neighbors.insert(masked_points.row(i));
    }
    
    return neighbors;

}

    
std::set<Eigen::RowVector3d,RowVector3dComparator> get_neighbors_butbetter(KDTreeEigenMatrixAdaptor<double>& kdtree,
    Eigen::RowVector3d loc, double resolution, Eigen::RowVector3d initial_pose, Eigen::MatrixXd& points)
    {    
        std::vector<size_t> neighbor_indices;
        std::vector<double> neighbor_distances;
        nanoflann::SearchParams search_params;
        search_params.sorted = true;

        const double search_radius = resolution * resolution;
        std::vector<std::pair<uint32_t, double>> matches;
        // std::cout<<"about to search"<<std::endl;
        // std::cout << "KD-tree contains " << kdtree.kdtree_get_point_count() << " points." << std::endl;
        const size_t num_found = kdtree.index->radiusSearch(loc.data(), search_radius, matches, search_params);
        // std::cout<<"found neighbors"<<std::endl;

        std::set<Eigen::RowVector3d, RowVector3dComparator> neighbors(initial_pose);

        for (const auto& match : matches) {
            size_t idx = match.first;
            double dist_sq = match.second;
    
            // Filter out self-matches (if applicable)
            if (dist_sq > 1e-3) {
                neighbors.insert(points.row(idx));
            }
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