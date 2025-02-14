#include <Eigen/Dense>
#include <iostream>

class BFS
{
    private:
        const Eigen::MatrixXd points;
        const Eigen::Matrix<double,1,3> initial_pose;
        const Eigen::Matrix<double,1,3> goal_pose;
        const double resolution;

    
    public:
        Eigen::MatrixXd get_neighbors(Eigen::Matrix<double,1,3> loc)
        {    
            Eigen::MatrixXd dist(points.rows(),points.cols());
            dist = points.rowwise() - loc;
            Eigen::MatrixXd new_dist(points.rows(),1);
            new_dist = dist.rowwise().norm();

            Eigen::MatrixXd mask = (new_dist.array() <= (resolution + 1e-2)).cast<double>();
            Eigen::MatrixXd masked_points(points.rows(),points.cols());
            for (int r=0;r<points.rows();r++)
            {
                Eigen::Matrix<double,1,3> row_mult = points.row(r) * mask(r,0);

                masked_points(r,0) = row_mult[0];
                masked_points(r,1) = row_mult[1];
                masked_points(r,2) = row_mult[2];
            }
            std::cout<<"Applied mask"<<std::endl;

            Eigen::MatrixXd nonzero_points(26,3); //max # of neighbors is 26

            int curr_row = 0;
            for (int i=0;i<masked_points.rows();i++)
            {
                if (masked_points(i,0) > 1e-6 || masked_points(i,1) > 1e-6 || masked_points(i,2) > 1e-6) 
                { 
                    nonzero_points(curr_row,0) = masked_points(i,0);
                    nonzero_points(curr_row,1) = masked_points(i,1);
                    nonzero_points(curr_row,2) = masked_points(i,2);
                    curr_row++;
                    if (curr_row > 25){ break; }
                }

            }

            if (curr_row == 0){
                std::cout<<"no neighbors"<<std::endl;
            }

            return nonzero_points;
        }

        BFS (const Eigen::Matrix<double,1,3> initial, const Eigen::Matrix<double,1,3> goal, Eigen::MatrixXd pts, double res)
         :  points(pts),initial_pose(initial),goal_pose(goal), resolution(res)
        {
            std::cout<<"resolution: "<<resolution<<std::endl;
            Eigen::Matrix<double,1,3> m(0,0,0);
            std::cout<<get_neighbors(m)<<std::endl;

        }
};