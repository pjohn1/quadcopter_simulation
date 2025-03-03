#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <vector>
#include <nlopt.h>
#include <numeric>
#include <vector>

using namespace boost::numeric::odeint;

class Solver
{
    private:
        int N;
        double m;
        double Iyy;
        void update_state(const std::vector<double>& x, std::vector<double> &dx, double t, double u, double Fz)
        {
            dx[0] = x[1];
            dx[1] = 1/m*Fz*sin(x[2]);
            dx[2] = x[3];
            dx[3] = u/Iyy;
        }

        double cost_fn(std::vector<double> u, std::Vector<double> current, std::vector<double> goal)
        //current is x,vx,pitch,pitch rate
        {
            double cost = 0.0;
            double dt = 0.05;
            double g = 9.81;
            cost = 0;
            for (int i=0;i<N;++i)
            {
                double Fz = m*g*cos(current[2]);
                double u_k = u[i];
                auto system = std::bind(update_state, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, u_k, Fz);
                integrate(system, current, 0.0, dt * 10, dt);
                Eigen::Map<Eigen::VectorXd> c(current.data(), current.size());
                Eigen::Map<Eigen::VectorXd> g(goal.data(), goal.size());
                Eigen::Vector diff = g-c;
                cost += u_k**2 + (diff.norm())**2;
            }
            return cost;
        }
    public:
        Solver(int horizon, Drone *drone) : N(horizon), m(drone->mass), Iyy(drone->inertia_tensor(1,1))
        {}

        double solve(Eigen::Vector pose, Eigen::Vector goal)
        {
            Eigen::vector<double,N> u;
            std::vector<Eigen::Vector> params = {u,pose,goal};

            nlopt::opt opt(nlpopt::LN_NELDERMEAD, N);
            opt.set_min_objective(cost_fn,&params);
            opt.set_xtol_rel(1e-4);

            double minf;
            nlopt::result result = opt.optimize(u,minf);
            std::cout<<"final cost: "<<minf<<std::endl;
            return u[0];

        }

};