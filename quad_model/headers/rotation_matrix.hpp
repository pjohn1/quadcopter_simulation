#include <Eigen/Dense>

class RotationMatrix
{
    private:
        Eigen::Matrix<double,3,3> R3;
        Eigen::Matrix<double,3,3> R2;
        Eigen::Matrix<double,3,3> R1;
    public:
        Eigen::Matrix<double,3,3> R;
        RotationMatrix(double roll,double pitch,double yaw)
        {
                    R3 << std::cos(yaw), -std::sin(yaw), 0,
                          std::sin(yaw), std::cos(yaw), 0,
                          0, 0, 1;
                    
                    R2 << std::cos(pitch), 0, std::sin(pitch),
                          0, 1, 0,
                          -std::sin(pitch), 0, std::cos(pitch);
                    
                    R1 << 1, 0, 0,
                          0, std::cos(roll), std::sin(roll),
                          0, -std::sin(roll), std::cos(roll);
                    
                    R = R3*R2*R1; //convert body attitude to inertial frame
        };
};