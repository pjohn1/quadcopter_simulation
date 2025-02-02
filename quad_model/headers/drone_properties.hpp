#include <Eigen/Dense>


class Drone {
    public:
        const double mass = 0.08; //kg
        Eigen::Matrix<double,3,3> inertia_tensor;
        const double distance_to_motor = 98.0/1000; //98 mm
        const double cd = 0.8;
        // inertia_tensor << .00679, 0, 0,
        //           0, .00679, 0,
        //           0, 0, , .01313;
        Drone()
        {
            inertia_tensor << .00679, 0.0, 0.0,
                      0.0, .00679, 0.0,
                      0.0, 0.0, .01313;
        }
        ~Drone(){}

};