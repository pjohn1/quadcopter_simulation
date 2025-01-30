#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include "drone_properties.hpp"
#include <iostream>

#define g 9.81

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
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;

        double desired_vx,desired_vy,desired_vz,desired_yaw_rate; //desired values
        double x,y,z = 0.0;
        double vx,vy,vz,yaw_rate = 0.0;
        double yaw,pitch,roll = 0.0;
        double yaw_new,pitch_new,roll_new = 0.0;
        double wx,wy,wz = 0.0;

        double last_w_error = 0.0;
        // const double dt=0.05; //20Hz
        double update_rate;
        double dt;
        double last_time;
        bool initialized = false;
        Drone *mass_prop;

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

        auto pose_callback = [this](const geometry_msgs::msg::Pose &msg) -> void
        {
            if (initialized)
            {
                double x_new = msg.position.x;
                double y_new = msg.position.y;
                double z_new = msg.position.z;
                Eigen::Matrix<double,1,3> pose(x,y,z);
                std::vector<double> forces = {0.0,0.0,0.0,0.0};
                //vx,vy,vz,yaw rate (CCW)
                dt = this->get_clock()->now().seconds() - last_time;
                std::cout<<"dt: "<<dt<<std::endl;
                vx = (x_new-x)/dt;vy=(y_new-y)/dt;vz=(z_new-z)/dt;
                x=x_new;y=y_new;z=z_new;


                Eigen::Quaterniond q2(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
                Eigen::Vector3d euler = q2.toRotationMatrix().eulerAngles(2,1,0);
                yaw = euler[2];pitch=-euler[1];roll=euler[0];
                double diff_roll = roll_new-roll;double diff_pitch = pitch_new-pitch;double diff_yaw=yaw_new-yaw;
                wx = (diff_roll)/dt;wy=-(diff_pitch)/dt;wz=(diff_yaw)/dt;

                // vz control
                // required mg is 0.7848N or 0.1962 from each motor
                double vz_error = desired_vz - vz;
                double f_i = mass_prop->mass/std::cos(pitch) * (vz_error/(update_rate) + g); //Fz formula
                std::cout<<"vz error: "<<vz_error<<std::endl;
                std::cout<<"force correction: "<<f_i<<std::endl;

                for (auto &force : forces) { force+=f_i/4; }

                double convergence_time = update_rate;

                //vx controller
                const double kp_vx = this->get_parameter("kp_vx").as_double(); // v = wr = sqrt(2)/2 * d * w
                const double kd_vx = this->get_parameter("kd_vx").as_double();
                double vx_error = desired_vx - vx;
                double desired_pitch = std::asin(mass_prop->mass*vx_error/(f_i*convergence_time));
                double pitch_error = desired_pitch - pitch;

                double desired_w = pitch_error/dt;
                double error = desired_w - wy;
                double derror = (error - last_w_error)/dt;
                last_w_error = error;

                std::cout<<"desired angular velocity: "<<desired_w<<" actual: "<<wy<<std::endl;

                double required_torque = mass_prop->inertia_tensor(1,1)*error/convergence_time;
                double correction_torque = mass_prop->inertia_tensor(1,1)*derror/convergence_time;

                double f = required_torque/(2*mass_prop->distance_to_motor);
                double df = correction_torque/(2*mass_prop->distance_to_motor);
                double f_new = kp_vx*f + kd_vx*df;
                std::cout<< "p: "<<kp_vx*f<<"d: "<<kd_vx*derror<<std::endl;

                
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
                // double error = desired_vx - vx;
                // double derror = error/dt;

                // double force_x = mass_prop->mass * error/dt; //required force to get to this deltav
                // fx = fz*sin(theta)






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
        this->declare_parameter<double>("kp_vx", 0.1);
        this->declare_parameter<double>("kd_vx",0.02);
        mass_prop = new Drone();
        velocity_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/velocities",1,velocity_callback);
        pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("/quad_pose",1,pose_callback);
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