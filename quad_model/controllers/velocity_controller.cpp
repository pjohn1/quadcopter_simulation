#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include "../headers/drone_properties.hpp"
#include <iostream>
#include "../headers/rotation_matrix.hpp"

#define MAX_PITCH 15*M_PI/180
#define MAX_ROLL 15*M_PI/180
#define MIN_MOTOR_FORCE 1e-3
#define g 9.81

/* This node takes a desired speed input and
 calculates the corresponding motor forces
 to accomplish that speed within the update rate
 timeframe at a constant angle (i.e. 10deg) */

void fixEuler(double &angle)
/*fixes strange outputs from quaternion to euler*/
{
    if (abs(angle - M_PI) < 1e-6)
    {
        angle = 0;
    }
    else if (abs(angle) < 1e-6)
    {
        angle = 0;
    }
    else if (abs(angle) > M_PI/2)
    {
        angle -= M_PI;
    }
}

class VelocityConverter : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr voltage_publisher;
        std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub;

        double desired_vx,desired_vy,desired_vz,desired_yaw_rate; //desired values
        double x,y,z = 0.0;
        double vx,vy,vz,yaw_rate = 0.0;
        double yaw,pitch,roll = 0.0;
        double yaw_new,pitch_new,roll_new = 0.0;
        double wx,wy,wz = 0.0;

        double last_w_error = 0.0;
        double last_wx_error = 0.0;

        double derror_wx=0.0;
        double derror=0.0;
        
        double ierror_wx = 0.0;
        double ierror=0.0;
        double if_ = 0.0;
        double if_x = 0.0;

        double dt;
        double last_time;
        bool initialized = false;
        double convergence_time;

        Drone *mass_prop;
        RotationMatrix *rotate;
        Eigen::Matrix<double,3,3> R;

    public:

    VelocityConverter() : Node("velocity_converter")
    {
        auto velocity_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
        {
            desired_vx=msg.data[0];desired_vy=msg.data[1];desired_vz=msg.data[2];desired_yaw_rate=msg.data[3];
            
            if (!initialized)
            {
                dt = this->get_parameter("update_rate").as_double();
            }
            initialized=true;
        };

        auto pose_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
        {
            //stopped using quaternions because of inconsistent angle wrapping issues
            //ie nonsingular vals (sometimes 0 rad would initialize to PI) causes subtraction issues
            // [x,y,z,roll,pitch,yaw]
            if (initialized)
            {
                double x_new = static_cast<double>(msg.data[0]);
                double y_new = static_cast<double>(msg.data[1]);
                double z_new = static_cast<double>(msg.data[2]);
                double roll_new = -static_cast<double>(msg.data[3]);
                // one of my frames is messed up but im too scared to touch the code lol
                double pitch_new = static_cast<double>(msg.data[4]);
                double yaw_new = static_cast<double>(msg.data[5]);
                std::vector<double> forces = {0.0,0.0,0.0,0.0};

                convergence_time = this->get_parameter("convergence_time").as_double();

                // dt = this->get_clock()->now().seconds() - last_time;
                vx = (x_new-x)/dt;vy=(y_new-y)/dt;vz=(z_new-z)/dt;
                x=x_new;y=y_new;z=z_new;
                Eigen::Matrix<double,1,3> pose(x,y,z);
                //updating velocities and angular velocities
                double diff_roll = roll_new-roll;double diff_pitch = pitch_new-pitch;double diff_yaw=yaw_new-yaw;
                wx = (diff_roll)/dt;wy=(diff_pitch)/dt;wz=(diff_yaw)/dt;
                yaw=yaw_new;pitch=pitch_new;roll=roll_new;

                rotate = new RotationMatrix(roll,pitch,yaw);
                R = rotate->R;

                // vz control
                // required mg is 0.7848N or 0.1962 from each motor
                double vz_error = desired_vz - vz;
                double rhs = mass_prop->mass*(vz_error/convergence_time+g);
                // required force in inertial frame to minimize z error

                Eigen::Matrix<double,3,1> F(0.0,0.0,rhs);
                Eigen::Matrix<double,3,1> f_m = R.inverse() * F;
                // convert inertial frame required force to body frame
                
                double f_i = f_m(2,0); //Fz in the body frame
                for (auto &force : forces) { force+=f_i/4; }
                // assume equal contribution from each motor

                //vx controller
                const double kp = this->get_parameter("kp").as_double();
                const double ki = this->get_parameter("ki").as_double();
                const double kd = this->get_parameter("kd").as_double();
                double vx_error = desired_vx - vx;
                // std::cout<<"vx error: "<<vx_error<<std::endl;

                double desired_pitch = std::asin(mass_prop->mass*vx_error/(f_i*convergence_time));
                if ( abs(desired_pitch) > MAX_PITCH )
                {
                    //maintain directionality
                    desired_pitch = desired_pitch/abs(desired_pitch) * MAX_PITCH;
                }

                else if ( std::isnan(desired_pitch) )
                {
                    //if arcsin throws a nan value, the pitch is too high so just pitch max
                    desired_pitch = vx_error/abs(vx_error) * MAX_PITCH;
                }
                // std::cout<<"contoller pitch called"<<std::endl;

                double pitch_error = desired_pitch - pitch;

                double desired_w = pitch_error/dt; //w = dtheta/dt for small dt
                double error = desired_w - wy;

                if (last_w_error != 0.0) //ensure last_error has been initialized
                {
                    derror = (error - last_w_error)/dt;
                    ierror = (error - last_w_error)*dt;
                }
                last_w_error = error;

                double required_torque = mass_prop->inertia_tensor(1,1)*error/convergence_time;
                // required torque to reach desired angular velocity in convergence time
                double correction_torque = mass_prop->inertia_tensor(1,1)*derror/convergence_time;
                double summed_torque = mass_prop->inertia_tensor(1,1)*ierror/convergence_time;

                double f = required_torque/(4*mass_prop->distance_to_motor);
                //divide by 4 because we add to necessary rotors and subtract from unnecessary rotors
                //ie if f=+4, we add +1 to back rotors and subtract 1 from front so (2) - (-2) = 4
                double df = correction_torque/(4*mass_prop->distance_to_motor);
                if_ += summed_torque/(4*mass_prop->distance_to_motor);

                double f_new = kp*f + ki*if_ + kd*df;
                
                if (error > 0.0)
                {
                    forces[2]+=f_new;forces[3]+=f_new;
                    forces[0]-=f_new;forces[1]-=f_new; //need to subtract to maintain force balance
                }
                else
                {
                    forces[0]+=abs(f_new);forces[1]+=abs(f_new);
                    forces[2]-=abs(f_new);forces[3]-=abs(f_new);
                }

                //vy controller
                //same as vx, probably could all be put into a template but i like explicit code
                double vy_error = desired_vy - vy;
                double desired_roll = std::asin(mass_prop->mass*vy_error/(f_i*convergence_time));
                if ( abs(desired_roll) > MAX_ROLL )
                {
                    desired_roll = desired_roll/abs(desired_roll) * MAX_ROLL;
                }

                else if ( std::isnan(desired_roll) )
                {
                    desired_roll = vy_error/abs(vy_error) * MAX_ROLL;
                }
                double roll_error = desired_roll - roll;

                // std::cout<<"desired roll: "<<desired_roll<<" actual: "<<roll<<std::endl;

                double desired_wx = roll_error/dt;
                double error_wx = desired_wx - wx;
                if (last_wx_error != 0.0)
                {
                    derror_wx = (error_wx - last_wx_error)/dt;
                    ierror_wx = (error_wx - last_wx_error)*dt;
                }
                last_wx_error = error_wx;

                double required_torque_wx = mass_prop->inertia_tensor(0,0)*error_wx/convergence_time;
                double correction_torque_wx = mass_prop->inertia_tensor(0,0)*derror_wx/convergence_time;
                double summed_torque_wx = mass_prop->inertia_tensor(0,0)*ierror_wx/convergence_time;

                double f_x = required_torque_wx/(4*mass_prop->distance_to_motor);
                double df_x = correction_torque_wx/(4*mass_prop->distance_to_motor);
                if_x += summed_torque_wx/(4*mass_prop->distance_to_motor);
                double f_new_x = kp*f_x + ki*if_x + kd*df_x;
                
                if (error_wx > 0.0)
                {
                    forces[1]+=f_new_x;forces[2]+=f_new_x;
                    forces[0]-=f_new_x;forces[3]-=f_new_x; //need to subtract to maintain vertical force balance
                }
                else
                {
                    forces[0]+=abs(f_new_x);forces[3]+=abs(f_new_x);
                    forces[1]-=abs(f_new_x);forces[2]-=abs(f_new_x);
                }

                //apply correction to ensure no negative motor forces
                Eigen::Matrix<double,1,4> pf(forces[0],forces[1],forces[2],forces[3]);
                double min_force = *std::min_element(forces.begin(),forces.end());
                double sum = pf.sum();
                if (min_force < MIN_MOTOR_FORCE)
                {
                    double scale = sum / ( sum + 4*abs(min_force) ); //sum scale factor
                    for(auto &force : forces) {
                        force += abs(min_force);
                        force *= scale;
                    }
                }

                // std::cout<<"Forces: "<<forces[0]<<" "<<forces[1]<<" "<<forces[2]<<" "<<forces[3]<<std::endl;
                std::vector<float> forces_float;
                for(auto &force : forces) {forces_float.push_back(static_cast<float>(force));}

                std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
                msg_pub.data = forces_float;
                voltage_publisher->publish(msg_pub);
                last_time = this->get_clock()->now().seconds();
            }
        };


        this->declare_parameter<double>("convergence_time",0.5);
        convergence_time = this->get_parameter("convergence_time").as_double();
        this->declare_parameter<double>("update_rate",0.0);
        this->declare_parameter<double>("kp", 4.0*convergence_time);
        this->declare_parameter<double>("ki", .33*convergence_time);
        this->declare_parameter<double>("kd",2.0*convergence_time);
        mass_prop = new Drone();
        velocity_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/velocities",2,velocity_callback);
        pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/quad_pose",2,pose_callback);
        voltage_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/voltage_input",2);
    }
    ~VelocityConverter()
    {
        delete rotate;
        delete mass_prop;
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