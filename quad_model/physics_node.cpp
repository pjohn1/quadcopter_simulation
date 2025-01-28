#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <Eigen/Dense>
#include "drone_properties.hpp"

class ForcePubSub : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        Drone *mass_prop = new Drone();

        rclcpp::Time last_time;

        Eigen::Matrix3d inertia_tensor;
        Eigen::Matrix<double,3,3> R3;
        Eigen::Matrix<double,3,3> R2;
        Eigen::Matrix<double,3,3> R1;
        Eigen::Matrix<double,3,3> R; //rotation matrix from inertial to body frame
        double mass;
        double dt;

        Eigen::Matrix<double,3,3> Rbn;
        double wx,wy,wz;
        double vx,vz,vy;           // Vertical velocity
        
    public:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr getPublisher(){ return publisher_; }
        ForcePubSub() : Node("force_pub_sub")
        {
            auto voltage_callback = [this](std_msgs::msg::Float32MultiArray msg) -> void
            {
                // std::cout<<"received voltage"<<std::endl;
                const double k = 1.0; //thrust coefficient
                const double d = mass_prop->distance_to_motor; //distance from center to rotor
                const double mass = mass_prop->mass;
                inertia_tensor = mass_prop->inertia_tensor;
                rclcpp::Time t = this->get_clock()->now();

                std::vector<float> v_ = msg.data;  //motors are TL TR BL BR
                std::vector<double> v(v_.begin(),v_.end());
                
                Eigen::ArrayXd voltages = Eigen::Map<Eigen::ArrayXd>(v.data(), v.size());
                // std::cout << voltages << std::endl;
                for(auto &val : voltages) { val = k*std::pow(val,2); }

                Eigen::ArrayXd &forces = voltages;
                
                float Fz = forces.sum();  //upwards force is just sum of all the forces;
                float Tx = d * (forces[0] + forces[3] - forces[1] - forces[2]); //roll (TL + BR) - (TR + BL)
                float Ty = d * -(forces[0] + forces[1] - forces[2] - forces[3]); //pitch (TL + TR) - (BL + BR)
                float Tz = (forces[0] + forces[2] - forces[1] - forces[3]); //yaw (TL + BR) - (TR + BL)

                if (last_time.seconds() != 0.0)
                {
                    dt = t.seconds() - last_time.seconds();

                    /* Calculating attitude using first-order Rodrigues rotation formula*/
                    /* https://www.roboticsbook.org/S72_drone_actions.html#drone-kinematics */
                    Eigen::Matrix<double,1,3> torques;
                    torques << Tx,Ty,Tz;
                    Eigen::Matrix<double,1,3> angular_velocity = torques * inertia_tensor.inverse();
                    wx=angular_velocity[0];wy=angular_velocity[1];wz=angular_velocity[2];


                    Eigen::Matrix<double,3,3> Rbn_next;
                    Rbn_next << 1, -wz*dt, wy*dt,
                                wz*dt, 1, -wx*dt,
                                -wy*dt, wx*dt, 1;
                    Rbn = Rbn * Rbn_next; //attitude matrix
                    //first column -> pitch angle;second ->roll angle;third->yaw angle;
                    double yaw_angle = std::atan2(Rbn(1,0),Rbn(0,0)); //xx/xy
                    double pitch_angle = std::atan2(-Rbn(2,0),std::sqrt(std::pow(Rbn(2,1),2) + std::pow(Rbn(2,2),2) )); //zz/zx
                    double roll_angle = std::atan2(Rbn(2,1),Rbn(2,2)); //zz/zy

                    //pitch is increasing from 
                    // std::cout<<pitch_angle<<std::endl;

                    R3 << std::cos(yaw_angle), -std::sin(yaw_angle), 0,
                          std::sin(yaw_angle), std::cos(yaw_angle), 0,
                          0, 0, 1;
                    
                    R2 << std::cos(pitch_angle), 0, std::sin(pitch_angle),
                          0, 1, 0,
                          -std::sin(pitch_angle), 0, std::cos(pitch_angle);
                    
                    R1 << 1, 0, 0,
                          0, std::cos(roll_angle), -std::sin(roll_angle),
                          0, std::sin(roll_angle), std::cos(roll_angle);
                    
                    R = R3*R2*R1;
                    // std::cout<<R3<<R2<<R1<<std::endl;

                    Eigen::Matrix<double,3,1> Fb;
                    Fb << 0,0,Fz;

                    Eigen::Matrix<double,3,1> an = R*Fb/mass;//( (R*Fb) - Eigen::Matrix<double,3,1>(0.0,0.0,mass*9.81) )/mass; //inertial frame forces
                    Eigen::Matrix<double,3,1> vn = an*dt; //linear velocity
                    vx=vn[0];vy=vn[1];vz=vn[2];

                    std::vector<double> double_vals = {vx,vy,vz,wx,wy,wz};
                    std::vector<float> msg_data;
                    for( auto &val : double_vals) { msg_data.push_back(static_cast<float>(val));}

                    std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
                    msg_pub.data = msg_data;
                    // std::cout << "vx:" << vx << std::endl;
                    // std::cout << "vz:" << vz << std::endl;
                    // std::cout << "wx:" << wx << std::endl;
                    publisher_->publish(msg_pub);

                }
                else { //assuming it starts on level ground
                    vz = forces.sum()/mass;
                    float msg_vz = static_cast<float>(vz);
                    std::vector<float> msg_data = {0,0,msg_vz,0,0,0};
                    std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
                    msg_pub.data = msg_data;
                    publisher_->publish(msg_pub);

                }
                last_time = t;
            };
            Rbn << 1,0,0,0,1,0,0,0,1;
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("voltage_input",1,voltage_callback);
            publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/quadcopter/forces",1);
        }
        ~ForcePubSub(){
            delete mass_prop;
        }
        
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto force_pub_sub = std::make_shared<ForcePubSub>();
    rclcpp::spin(force_pub_sub);
    rclcpp::shutdown();
    return 0;
}