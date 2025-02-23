#include <Eigen/Dense>
#include <set>
#include <iostream>
#include <vector>
#include <nanoflann.hpp>


struct PathNode {
    Eigen::RowVector3d pose;
    std::shared_ptr<PathNode> parent;
    double g_score;
    double f_score;
    bool has_parent;

    //default constructor for KDTree building and path retracing
    PathNode(Eigen::RowVector3d curr_pose): pose(curr_pose), has_parent(false),
    g_score(std::numeric_limits<double>::infinity()),f_score(std::numeric_limits<double>::infinity()){}

    //constructor when path costs are known
    PathNode(Eigen::RowVector3d curr_pose, std::shared_ptr<PathNode> last_node, double gscore, double fscore)
    : pose(curr_pose),parent(last_node),has_parent(true),g_score(gscore),f_score(fscore) 
    {}

    //constructor for initial pose object (no parent)
    PathNode(Eigen::RowVector3d curr_pose, double gscore, double fscore)
    : pose(curr_pose),has_parent(false),g_score(gscore),f_score(fscore) 
    {}
};

template <typename num_t>
struct KDTreePathNodeAdaptor {
    //Strongly aided by ChatGPT, hence the comments

    // Define the KDTree type with a 3-dimensional space and L2 distance.
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<num_t, KDTreePathNodeAdaptor<num_t>>,
        KDTreePathNodeAdaptor<num_t>,
        3
    > KDTree;

    std::vector<std::shared_ptr<PathNode>> nodes;
    KDTree* index;

    KDTreePathNodeAdaptor(const std::vector<std::shared_ptr<PathNode>>& nodes_)
        : nodes(nodes_) {
        index = new KDTree(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        index->buildIndex();
    }

    ~KDTreePathNodeAdaptor() { delete index; }

    // Return the number of nodes.
    inline size_t kdtree_get_point_count() const { return nodes.size(); }

    // Access the dim'th component of the pose for node at index idx.
    inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const {
        // Assuming that 'pose' is an Eigen::RowVector3d.
        return nodes[idx]->pose(dim);
    }

    // Optional bounding-box computation: return false to use a default bbox computation.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};


struct RowVector3dComparator {
    //sorts the nighbors by the x,y values (not super necessary tbh but useful for unit-length path search)
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
//really just used in comparison to see if a PathNode elt is in the seen set
// when the operator returns false, the two objects are equal otherwise sorts by pose
bool operator()(const PathNode& lhs, const PathNode& rhs) const {
    if ( lhs.pose[0] != rhs.pose[0] ) return lhs.pose[0] < rhs.pose[0];
    if ( lhs.pose[1] != rhs.pose[1]) return lhs.pose[1] < rhs.pose[1];
    if ( lhs.pose[2] != rhs.pose[2]) return lhs.pose[2] < rhs.pose[2];
    if (lhs.has_parent != rhs.has_parent) return lhs.has_parent;
    if ( lhs.parent->pose[0] != rhs.parent->pose[0]) return lhs.parent->pose[0] < rhs.parent->pose[0];
    if ( lhs.parent->pose[1] != rhs.parent->pose[1]) return lhs.parent->pose[1] < rhs.parent->pose[1];
    if ( lhs.parent->pose[2] != rhs.parent->pose[2]) return lhs.parent->pose[2] < rhs.parent->pose[2];
    return abs(lhs.g_score - rhs.g_score) > 1e-6;
}
};

std::set<Eigen::RowVector3d, RowVector3dComparator> get_neighbors(Eigen::RowVector3d loc, Eigen::MatrixXd points,
double resolution, Eigen::RowVector3d initial_pose)
{    
    //old neighbors function, slow and not used in path finding but its nostalgic

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

    
std::vector<std::shared_ptr<PathNode>> get_neighbors_butbetter(KDTreePathNodeAdaptor<double>& kdtree,
    const Eigen::RowVector3d loc, double resolution, Eigen::RowVector3d initial_pose)
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

        std::vector<std::shared_ptr<PathNode>> neighbors;

        for (const auto& match : matches) {
            size_t idx = match.first; //index in node tree
            double dist_sq = match.second; //distance value
    
            if (dist_sq > 1e-3) {
                //do not keep current node
                neighbors.push_back(kdtree.nodes[idx]);
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