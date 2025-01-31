#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <Eigen/Dense>
#include "drone_properties.hpp"
#include "rotation_matrix.hpp"

#define g 9.81

class ForcePubSub : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        Drone *mass_prop = new Drone();

        double last_time=0.0;

        Eigen::Matrix3d inertia_tensor;
        double mass;
        double dt;

        Eigen::Matrix<double,3,3> Rbn;
        double wx,wy,wz=0.0;
        double vx,vz,vy=0.0;           // Vertical velocity

        Eigen::Matrix<double,3,3> R;
        RotationMatrix *rotate;
        
        
    public:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr getPublisher(){ return publisher_; }
        ForcePubSub() : Node("force_pub_sub")
        {
            auto voltage_callback = [this](std_msgs::msg::Float32MultiArray msg) -> void
            {
                // std::cout<<"received voltage"<<std::endl;
                const double d = mass_prop->distance_to_motor; //distance from center to rotor
                const double mass = mass_prop->mass;
                inertia_tensor = mass_prop->inertia_tensor;

                std::vector<float> v_ = msg.data;  //motors are TL TR BL BR
                std::vector<double> v(v_.begin(),v_.end());
                
                Eigen::ArrayXd voltages = Eigen::Map<Eigen::ArrayXd>(v.data(), v.size());
                // std::cout << voltages << std::endl;
                // for(auto &val : voltages) { val = k*std::pow(val,2); }

                Eigen::ArrayXd &forces = voltages;
                std::cout<<"Incoming forces: "<<forces<<std::endl;
                
                /* Body frame forces */
                float Fz = forces.sum();  //upwards force in body frame is just sum of all the forces;
                float Tx = d * (forces[1] + forces[2] - forces[0] - forces[3]); //roll (TL + BR) - (TR + BL)
                float Ty = d * (forces[2] + forces[3] - forces[0] - forces[1]); //pitch (BL + BR) - (TL + TR)
                float Tz = (forces[0] + forces[2] - forces[1] - forces[3]); //yaw (TL + BR) - (TR + BL)
                /**************************/

                if (last_time != 0.0)
                {
                    /* Calculating attitude using first-order Rodrigues rotation formula*/
                    /* https://www.roboticsbook.org/S72_drone_actions.html#drone-kinematics */        
                    dt = this->get_clock()->now().seconds() - last_time;
                    Eigen::Matrix<double,3,1> torques;
                    torques << Tx,Ty,Tz;
                    std::cout<<"Torques: "<<torques<<std::endl;
                    Eigen::Matrix<double,3,1> angular_velocity = (inertia_tensor.inverse() * torques) * dt;
                    wx+=angular_velocity[0];wy+=angular_velocity[1];wz+=angular_velocity[2];
                    Eigen::Matrix<double,1,3> w(wx,wy,wz);
                    std::cout<<"angular velocities: "<< w << std::endl;


                    Eigen::Matrix<double,3,3> Rbn_next;
                    Rbn_next << 1, -wz*dt, wy*dt,
                                wz*dt, 1, -wx*dt,
                                -wy*dt, wx*dt, 1;
                    Rbn = Rbn * Rbn_next; //attitude matrix
                    //first column -> pitch angle;second ->roll angle;third->yaw angle;
                    double yaw_angle = std::atan2(Rbn(1,0),Rbn(0,0)); //xx/xy
                    double pitch_angle = std::atan2(-Rbn(2,0),std::sqrt(std::pow(Rbn(2,1),2) + std::pow(Rbn(2,2),2) )); //zz/zx
                    double roll_angle = -std::atan2(Rbn(1,2),Rbn(2,2)); //zy/zz

                    std::cout<<"Roll angle: "<<roll_angle<<std::endl;

                    //pitch is increasing from 
                    // std::cout<<pitch_angle<<std::endl;

                    // std::cout<<R3<<R2<<R1<<std::endl;

                    Eigen::Matrix<double,3,1> Fb;
                    Fb << 0,0,Fz;
                    Eigen::Matrix<double,3,1> Fo;
                    //Fg acts in the inertial -z direction (will subtract)
                    //assume drone is essentially rectangular cross section
                    //gonna assume constant cross section bc small angles and yeah
                    double drag = 0;//1/2.0 * mass_prop->cd * 1.225 * .005 * pow(vx,2);
                    Fo << drag,0,mass*g;

                    rotate = new RotationMatrix(roll_angle,pitch_angle,yaw_angle);
                    R = rotate->R;

                    Eigen::Matrix<double,3,1> dvn = dt/mass * ( (R*Fb - Fo) ); //change in velocity
                    vx+=dvn[0];vy+=dvn[1];vz+=dvn[2];

                    std::cout<<"vy: "<<vy<<" wx: "<<wx<<std::endl;
                    std::vector<double> double_vals = {vx,vy,vz,wx,wy,wz};
                    std::vector<float> msg_data;
                    for( auto &val : double_vals) { msg_data.push_back(static_cast<float>(val));}

                    std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
                    msg_pub.data = msg_data;
                    publisher_->publish(msg_pub);
                    last_time = this->get_clock()->now().seconds();
                    
                }

                else { //assuming it starts on level ground
                    // vz = forces.sum() - mass*g;
                    vz=0.0;
                    float msg_vz = static_cast<float>(vz);
                    std::vector<float> msg_data = {0,0,msg_vz,0,0,0};
                    std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
                    msg_pub.data = msg_data;
                    publisher_->publish(msg_pub);
                    last_time = this->get_clock()->now().seconds();

                }
            };
            Rbn << 1,0,0,0,1,0,0,0,1;
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("voltage_input",1,voltage_callback);
            publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/quadcopter/forces",1);
        }
        ~ForcePubSub(){
            delete mass_prop;
            delete rotate;
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