#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include "drone_properties.hpp"

class VelocityConverter : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr voltage_publisher;
        double vx,vy,vz,yaw_rate;
        const double dt=0.05; //20Hz
        Drone *mass_prop;

    public:

    VelocityConverter() : Node("velocity_converter")
    {
        auto velocity_callback = [this](std_msgs::msg::Float32MultiArray &msg) -> void
        {
            vx=msg.data[0];vy=msg.data[1];vz=msg.data[2];yaw_rate=msg.data[3];
            std::vector<float> forces = {0.0,0.0,0.0,0.0};

            double force_x = mass_prop->inertia_tensor(0,0) * vx * 2 * dt/pow(mass_prop->distance_to_motor,2); //per prop (assuming two prop)
            //inertia_tensor is symmetric so can just use corresponding principal axis
            double force_y = mass_prop->inertia_tensor(1,1) * vy * 2 * dt/pow(mass_prop->distance_to_motor,2); //per prop (assuming two prop)
            double force_z = mass_prop->mass*vz*dt / 4; //per propeller (assuming two propellors)
            double force_yaw = mass_prop->inertia_tensor(2,2) * yaw_rate * dt/2; //per prop

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

            if (yaw_rate > 0.0)
            {
                forces[0] += force_yaw;
                forces[2] += force_yaw;

            }
            else {
                forces[1] += force_yaw;
                forces[3] += force_yaw;
            }


            for (auto &f : forces) { f+= force_z;}

            std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
            msg_pub.data = forces;
            voltage_publisher->publish(msg_pub);
            
        };
        mass_prop = new Drone();
        velocity_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/velocities",1,velocity_callback);
        voltage_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/voltage_input",1);
        
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