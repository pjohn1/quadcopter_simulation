#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include "../headers/drone_properties.hpp"
#include <iostream>
#include "../headers/rotation_matrix.hpp"

#define MAX_PITCH 15*M_PI/180
#define MAX_ROLL 15*M_PI/180

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
        // const double dt=0.05; //20Hz
        double update_rate;
        double dt;
        double last_time;
        bool initialized = false;
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
                last_time = this->get_clock()->now().seconds();
            }
            initialized=true;
        };

        auto pose_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
        {
            //stopped using quaternions because of inconsistent angle wrapping issues
            // [x,y,z,roll,pitch,yaw]
            if (initialized)
            {
                double x_new = static_cast<double>(msg.data[0]);
                double y_new = static_cast<double>(msg.data[1]);
                double z_new = static_cast<double>(msg.data[2]);
                double roll_new = -static_cast<double>(msg.data[3]);
                double pitch_new = static_cast<double>(msg.data[4]);
                double yaw_new = static_cast<double>(msg.data[5]);
                std::vector<double> forces = {0.0,0.0,0.0,0.0};

                dt = this->get_clock()->now().seconds() - last_time;
                vx = (x_new-x)/dt;vy=(y_new-y)/dt;vz=(z_new-z)/dt;
                x=x_new;y=y_new;z=z_new;
                Eigen::Matrix<double,1,3> pose(x,y,z);

        
                // Eigen::Quaterniond q2(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
                // Eigen::Vector3d euler = q2.toRotationMatrix().eulerAngles(2,1,0);
                // yaw_new = euler[2];pitch_new=-euler[1];roll_new=euler[0];
                // fixEuler(pitch_new);fixEuler(roll_new);
                // //make sure the wrapping is consistent

                double diff_roll = roll_new-roll;double diff_pitch = pitch_new-pitch;double diff_yaw=yaw_new-yaw;
                wx = (diff_roll)/dt;wy=(diff_pitch)/dt;wz=(diff_yaw)/dt;
                yaw=yaw_new;pitch=pitch_new;roll=roll_new;

                rotate = new RotationMatrix(roll,pitch,yaw);
                R = rotate->R;
                // std::cout<<R<<std::endl;

                // vz control
                // required mg is 0.7848N or 0.1962 from each motor
                double vz_error = desired_vz - vz;
                double rhs = mass_prop->mass*(vz_error/update_rate+g);
                Eigen::Matrix<double,3,1> F(0.0,0.0,rhs);
                // double f_i = mass_prop->mass/std::cos(pitch) * (vz_error/(update_rate) + g); //Fz formula
                std::cout<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw<<std::endl;
                Eigen::Matrix<double,3,1> f_m = R.inverse() * F;
                std::cout<<f_m<<std::endl<<std::endl;
                double f_i = f_m(2,0);
                for (auto &force : forces) { force+=f_i/4; }

                double convergence_time = update_rate;

                //vx controller
                const double kp_vx = this->get_parameter("kp_vx").as_double(); // v = wr = sqrt(2)/2 * d * w
                const double kd_vx = this->get_parameter("kd_vx").as_double();
                double vx_error = desired_vx - vx;
                std::cout<<"vx error: "<<vx_error<<std::endl;
                double desired_pitch = std::asin(mass_prop->mass*vx_error/(f_i*convergence_time));
                if ( abs(desired_pitch) > MAX_PITCH )
                {
                    desired_pitch = desired_pitch/abs(desired_pitch) * MAX_PITCH;
                }

                else if ( std::isnan(desired_pitch) )
                {
                    desired_pitch = vx_error/abs(vx_error) * MAX_PITCH;
                }

                double pitch_error = desired_pitch - pitch;
                // std::cout<<"desired pitch: "<<desired_pitch<<std::endl;

                double desired_w = pitch_error/dt;
                double error = desired_w - wy;
                // std::cout<<"desired_w: "<<desired_w<<" wy: "<<error<<std::endl;
                if (last_w_error != 0.0)
                {
                    derror = (error - last_w_error)/dt;
                    // std::cout<<"derror "<<derror<<std::endl;
                }
                last_w_error = error;

                double required_torque = mass_prop->inertia_tensor(1,1)*error/convergence_time;
                double correction_torque = mass_prop->inertia_tensor(1,1)*derror/convergence_time;

                double f = required_torque/(4*mass_prop->distance_to_motor);
                //divide by 4 because we add to necessary rotors and subtract from unnecessary rotors
                double df = correction_torque/(4*mass_prop->distance_to_motor);
                double f_new = kp_vx*f + kd_vx*df;
                std::cout<<"px: "<<kp_vx*f<<" dx: "<<kd_vx*df<<std::endl;
                
                if (error > 0.0)
                {
                    forces[2]+=f_new;forces[3]+=f_new;
                    forces[0]-=f_new;forces[1]-=f_new; //need to subtract to maintain vertical force balance
                }
                else
                {
                    forces[0]+=abs(f_new);forces[1]+=abs(f_new);
                    forces[2]-=abs(f_new);forces[3]-=abs(f_new);
                }

                //vy controller
                const double kp_vy = this->get_parameter("kp_vy").as_double(); // v = wr = sqrt(2)/2 * d * w
                const double kd_vy = this->get_parameter("kd_vy").as_double();
                double vy_error = desired_vy - vy;
                double desired_roll = std::asin(mass_prop->mass*vy_error/(f_i*convergence_time));
                // double desired_roll = 5*M_PI/180;
                if ( abs(desired_roll) > MAX_ROLL )
                {
                    desired_roll = desired_roll/abs(desired_roll) * MAX_ROLL;
                }

                else if ( std::isnan(desired_roll) )
                {
                    desired_roll = vy_error/abs(vy_error) * MAX_ROLL;
                }
                double roll_error = desired_roll - roll;

                std::cout<<"desired roll: "<<desired_roll<<" actual: "<<roll<<std::endl;

                double desired_wx = roll_error/dt;
                double error_wx = desired_wx - wx;
                if (last_wx_error != 0.0) //initialization causes issues sometimes
                {
                    derror_wx = (error_wx - last_wx_error)/dt;
                }
                last_wx_error = error_wx;

                double required_torque_wx = mass_prop->inertia_tensor(0,0)*error_wx/convergence_time;
                double correction_torque_wx = mass_prop->inertia_tensor(0,0)*derror_wx/convergence_time;

                double f_x = required_torque_wx/(4*mass_prop->distance_to_motor);
                double df_x = correction_torque_wx/(4*mass_prop->distance_to_motor);
                double f_new_x = kp_vy*f_x + kd_vy*df_x;
                std::cout<< "p_y: "<<kp_vy*f_x<<" d_y: "<<kd_vy*df_x<<std::endl;

                
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

                std::vector<float> forces_float;
                for(auto &force : forces) {forces_float.push_back(static_cast<float>(force));}

                std_msgs::msg::Float32MultiArray msg_pub = std_msgs::msg::Float32MultiArray();
                msg_pub.data = forces_float;
                voltage_publisher->publish(msg_pub);
                last_time = this->get_clock()->now().seconds();
            }
        };


        this->declare_parameter<double>("update_rate", 10.0);
        update_rate = 1.0/this->get_parameter("update_rate").as_double();
        this->declare_parameter<double>("kp_vx", 0.3);
        this->declare_parameter<double>("kd_vx",0.03);
        this->declare_parameter<double>("kp_vy", 0.2);
        this->declare_parameter<double>("kd_vy",0.03);
        mass_prop = new Drone();
        velocity_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/velocities",1,velocity_callback);
        pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/quad_pose",1,pose_callback);
        voltage_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/voltage_input",1);
        parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "source_node");
    }
    ~VelocityConverter()
    {
        delete rotate;
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


/***TORQUE STUFF */

// // double desired_pitch = std::asin(mass_prop->mass*vx_error/(f_i*convergence_time)); //desired roll to reach velocity in one timestep
// // double dpitch = 
// double desired_pitch = vx_error*dt*sqrt(2)/mass_prop->distance_to_motor;
// double ddesired = derror*dt*sqrt(2)/mass_prop->distance_to_motor;

// // double control_law = kp_vx*desired_pitch;
// std::cout<<"desired pitch: "<<desired_pitch<<std::endl;
// double required_torque = mass_prop->inertia_tensor(1,1)*(desired_pitch)/pow(convergence_time,2);
// double drequired_torque = mass_prop->inertia_tensor(1,1)*(ddesired)/pow(convergence_time,2);
// std::cout<<"required torque: "<<required_torque<<std::endl;
// //no need to use angle wrapping because the angle should always remain in quadrant 1/2
// double f_new = abs(required_torque)/(2*mass_prop->distance_to_motor);
// double df = abs(drequired_torque)/(2*mass_prop->distance_to_motor);
// f_new = kp_vx*f_new + kd_vx*df;
// last_vx_error = vx_error;
// double f_new = kp_vx*vx_error + kd_vx*derror;
// std::cout<<"P control: "<<kp_vx*vx_error<<std::endl;
// std::cout<<"D control: "<<kd_vx*derror<<std::endl;

// last_vx_error = vx_error;