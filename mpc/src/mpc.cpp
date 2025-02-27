#include <cmath>
#include "acados/utils/types.h"
#include "../headers/drone_properties.hpp"
#include "acados_c/external_function_interface.h"

#define g 9.81
#define NS 4
#define NU 2

const Drone *drone = new Drone();

void forward_dynamics( double *xdot, const double *x, const double *u)
{
    double x = x[0];
    double theta = x[1];
    double torque_y = u[0];
    double fz_b = u[1];

    xdot[0] = (1/drone->mass) * fz_b * sin(theta);
    xdot[1] = (drone->inertia_tensor).inverse() * torque_y;
}

std::vector<double*> solve_mpc()
{
    
}