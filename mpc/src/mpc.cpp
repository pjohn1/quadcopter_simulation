#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include "controller.h"
#include "../headers/drone_properties.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

Controller::Controller(){
    Py_Initialize();

    PyObject *sysPath = PySys_GetObject("path");
    PyList_Append(sysPath, PyUnicode_DecodeFSDefault("."));
    PyList_Append(sysPath, PyUnicode_DecodeFSDefault("/mnt/c/Desktop/quadcopter_simulation/mpc/src/"));


    PyObject *pName = PyUnicode_DecodeFSDefault("nonlinear_solver");  // Python script name (without .py)
    pModule = PyImport_Import(pName);
    std::cout<<"Imported module"<<std::endl;

    if (!pModule) {
        PyErr_Print();
        std::cerr << "Failed to load Python module 'nonlinear_solver'" << std::endl;
    }

    Py_XDECREF(pName);
    std::cout<<"getting module"<<std::endl;

    pFunc = PyObject_GetAttrString(pModule, "main");
    std::cout<<"Got pfunc"<<std::endl;

}

Controller::~Controller() {
    Py_XDECREF(pFunc);
    Py_XDECREF(pModule);
    Py_Finalize();
}

std::vector<double> Controller::optimizeControl(int N, const std::vector<double>& X0, const std::vector<double>& goal) {
    // Convert C++ variables to Python objects
    PyObject *pyN = PyLong_FromLong(N);
    PyObject *pyX0 = PyList_New(X0.size());
    PyObject *pyGoal = PyList_New(goal.size());

    for (size_t i = 0; i < X0.size(); ++i) {
        PyList_SetItem(pyX0, i, PyFloat_FromDouble(X0[i]));
    }
    for (size_t i = 0; i < goal.size(); ++i) {
        PyList_SetItem(pyGoal, i, PyFloat_FromDouble(goal[i]));
    }

    // Call the Python function
    PyObject *args = PyTuple_Pack(3, pyN, pyX0, pyGoal);
    PyObject *pyResult = PyObject_CallObject(pFunc, args);

    // Clean up
    Py_XDECREF(pyN);
    Py_XDECREF(pyX0);
    Py_XDECREF(pyGoal);
    Py_XDECREF(args);

    // Process result
    std::vector<double> results;
    if (pyResult) {
        Py_ssize_t tuple_size = PyTuple_Size(pyResult);
        for (Py_ssize_t i = 0; i < tuple_size; ++i) {
            PyObject *item = PyTuple_GetItem(pyResult, i);  // Borrowed reference
            if (PyFloat_Check(item)) {
                results.push_back(PyFloat_AsDouble(item));
            } else {
                std::vector<double> empty;
                return empty;
            }
        }
        Py_XDECREF(pyResult);
    } else {
        PyErr_Print();
        std::cerr << "Python function call failed." << std::endl;
    }

    return results;
}

class MPC : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr control_pub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;

        Controller *controller;
        Drone *drone;

        double dt;

        double x,y,z;
        double vx,vy,vz;
        double yaw,pitch,roll;
        double wx,wy,wz;

        double initialized = false;


    public:

        MPC() : Node("mpc")
        {
            auto pose_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
            {
                if (initialized)
                {
                    double x_new = static_cast<double>(msg.data[0]);
                    double y_new = static_cast<double>(msg.data[1]);
                    double z_new = static_cast<double>(msg.data[2]);
                    double roll_new = static_cast<double>(msg.data[3]);
                    double pitch_new = static_cast<double>(msg.data[4]);
                    double yaw_new = static_cast<double>(msg.data[5]);

                    vx = (x_new-x)/dt;vy=(y_new-y)/dt;vz=(z_new-z)/dt;
                    x=x_new;y=y_new;z=z_new;
                    //updating velocities and angular velocities
                    wx = (roll_new-roll)/dt;wy=(pitch_new-pitch)/dt;wz=(yaw_new-yaw)/dt;
                    yaw=yaw_new;pitch=pitch_new;roll=roll_new;

                    std::vector<double> state = {x,vx,pitch,wy};
                    std::vector<double> goal = {x+1.0,0.0,0.0,0.0};
                    int N = 3;

                    std::vector<double> results = controller->optimizeControl(N,state,goal);
                    if (results.size() > 0 && results[1] < 30)
                    {
                        double optimized_u = results[0]/drone->distance_to_motor;
                        std::cout<<"control: "<<optimized_u<<"cost: "<<results[1];

                        double Fzb = drone->mass*9.81*cos(roll);
                        
                        std::vector<float> forces = {0.0,0.0,0.0,0.0};
                        
                            if (optimized_u > 0)
                            {
                                forces[2] = static_cast<float>(optimized_u/4);
                                forces[3] = static_cast<float>(optimized_u/4);
                            }
                            else
                            {
                                forces[0] = abs(static_cast<float>(optimized_u/2));
                                forces[1] = abs(static_cast<float>(optimized_u/2));             
                            }

                            // for (auto &force : forces)
                            // {
                            //     force += Fzb/4;
                            // }

                            std_msgs::msg::Float32MultiArray forces_msg = std_msgs::msg::Float32MultiArray();
                            forces_msg.data = forces;
                            control_pub->publish(forces_msg);
                    }
                }
                else {
                    x = static_cast<double>(msg.data[0]);
                    y = static_cast<double>(msg.data[1]);
                    z = static_cast<double>(msg.data[2]);
                    roll = static_cast<double>(msg.data[3]);
                    pitch = static_cast<double>(msg.data[4]);
                    yaw = static_cast<double>(msg.data[5]);
                }
            };
            auto goal_callback = [this](const geometry_msgs::msg::PointStamped &msg) -> void
            {
                initialized = true;
            };
            
            pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/quad_pose",2,pose_callback);
            control_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/voltage_input",2);
            goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point",2,goal_callback);
            controller = new Controller();
            this->declare_parameter("update_rate",0.0);
            dt = this->get_parameter("update_rate").as_double();
            drone = new Drone();
            

            // std::vector<float> f = {0.0,0.0,0.0,0.0,0.0,0.0};
            // std_msgs::msg::Float32MultiArray fake_msg = std_msgs::msg::Float32MultiArray();
            // fake_msg.data = f;
            // pose_callback(fake_msg);


        }
};

int main(int argc, char* argv[])
{

    rclcpp::init(argc,argv);
    auto visualize_node = std::make_shared<MPC>();
    rclcpp::spin(visualize_node);
    rclcpp::shutdown();
    return 0;
}