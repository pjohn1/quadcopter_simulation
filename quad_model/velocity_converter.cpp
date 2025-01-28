#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include "drone_properties.hpp"
#include <iostream>

/* This node takes a desired speed input and
 calculates the corresponding motor forces
 to accomplish that speed within the update rate
 timeframe at a constant angle (i.e. 10deg) */

class VelocityConverter : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr voltage_publisher;
        std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
        double vx,vy,vz,yaw_rate;
        // const double dt=0.05; //20Hz
        double dt;
        Drone *mass_prop;

    public:

    VelocityConverter() : Node("velocity_converter")
    {
        auto velocity_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
        {
            vx=msg.data[0];vy=msg.data[1];vz=msg.data[2];yaw_rate=msg.data[3];
            std::vector<float> forces = {0.0,0.0,0.0,0.0};

            double force_x = mass_prop->inertia_tensor(0,0) * vx / (2 * dt * pow(mass_prop->distance_to_motor,2) ); //per prop (assuming two prop)
            //inertia_tensor is symmetric so can just use corresponding principal axis
            double force_y = mass_prop->inertia_tensor(1,1) * vy / (2 * dt * pow(mass_prop->distance_to_motor,2) ); //per prop (assuming two prop)
            double force_z = mass_prop->mass*vz/(4 * dt); //per propeller (assuming two propellors)
            double force_yaw = mass_prop->inertia_tensor(2,2) * yaw_rate / (2*dt); //per prop

            if (vx > 0.0)
            {
                forces[2] += force_x;
                forces[3] += force_x;
            }
            else {
                forces[0] += abs(force_x);
                forces[1] += abs(force_x);
            }

            if (vy > 0.0)
            {
                forces[1] += force_y;
                forces[2] += force_y;
            }
            else {
                forces[0] += abs(force_y);
                forces[3] += abs(force_y);
            }

            // if (yaw_rate > 0.0)
            // {
            //     forces[0] += force_yaw;
            //     forces[2] += force_yaw;

            // }
            // else {
            //     forces[1] += force_yaw;
            //     forces[3] += force_yaw;
            // }


            for (auto &f : forces) { f+= force_z;}

            std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
            msg_pub.data = forces;
            std::cout<<std::endl;
            std::cout<<"forces:"<<forces[0]<<std::endl;
            std::cout<<"forces:"<<forces[1]<<std::endl;
            std::cout<<"forces:"<<forces[2]<<std::endl;
            std::cout<<"forces:"<<forces[3]<<std::endl;
            voltage_publisher->publish(msg_pub);
            
        };
        this->declare_parameter<double>("update_rate", 10.0);
        dt = 1.0/this->get_parameter("update_rate").as_double();
        mass_prop = new Drone();
        velocity_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/velocities",1,velocity_callback);
        voltage_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/voltage_input",1);
        parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "source_node");
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto controller_node = std::make_shared<VelocityConverter>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}