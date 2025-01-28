#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "quad_model/drone_properties.hpp"

int main() {
    // Original std::vector<float>
    Drone *d = new Drone();
    Eigen::Matrix3d i = d->inertia_tensor;
    Eigen::ArrayXd T = Eigen::ArrayXd(3,1);
    T << 1,2,3;
    std::cout<<T<<std::endl;

    return 0;
}
