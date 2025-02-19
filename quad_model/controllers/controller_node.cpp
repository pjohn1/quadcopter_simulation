#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.1419
#define MAX_VELOCITY 2.0
#define GOAL_EPS 0.1 //within goal if we are within this epsilon
#define DIST_ABOVE 0.5 //distance above the goal we want to get before descending

class ControllerNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub;

        Eigen::Matrix<double,1,3> goal_pose;
        double x,y,z = 0.0;
        double last_x,last_y,last_z=0.0;
        double update_rate;
        double yaw,pitch,roll=0.0;
        double goal_yaw = 0.0;
        bool initialized = false;
        bool goal_updated;

        Eigen::Matrix<double,1,3> last_error;
        Eigen::Matrix<double,1,3> ierror;

        double kp_yaw = 0.05;
        double kp,kd,ki;
    public:
        double angle_difference(double goal, double current)
        {
            double diff = std::fmod(goal - current, 2 * M_PI);
            if (diff < 0) {
                diff += 2 * M_PI; // Ensure the result is in [0, 2π]
            }
            return diff;
        }

        ControllerNode() : Node("controller")
        {

            auto point_callback = [this](const geometry_msgs::msg::PointStamped &msg) -> void
            {
                goal_pose << msg.point.x, msg.point.y, msg.point.z;
                //initialize goal pose to 0.5m above
                goal_yaw = std::atan2(msg.point.y,msg.point.x);
                if (!initialized) 
                {
                    last_error << 0.0,0.0,0.0;
                    ierror << 0.0,0.0,0.0;
                }
                // last_error << 0.0,0.0,0.0;
                initialized = true;
                goal_updated = false;
            };

            auto pose_callback = [this](const std_msgs::msg::Float32MultiArray &msg) -> void
            {
                if (initialized)
                {
                    double x = static_cast<double>(msg.data[0]);
                    double y = static_cast<double>(msg.data[1]);
                    double z = static_cast<double>(msg.data[2]);
                    double yaw = static_cast<double>(msg.data[5]);
                    Eigen::Matrix<double,1,3> pose(x,y,z); //iniitialize pose matrix

                    std::vector<double> velocities = {0.0,0.0,0.0,0.0};
                    kp = this->get_parameter("kp").as_double();
                    ki = this->get_parameter("ki").as_double();
                    kd = this->get_parameter("kd").as_double();

                    double yaw_difference = angle_difference(goal_yaw,yaw);
                    float yaw_control = kp_yaw * static_cast<float>(yaw_difference);

                    Eigen::Matrix<double,1,3> error = goal_pose-pose;
                    Eigen::Matrix<double,1,3> derror = (error-last_error)/update_rate;
                    ierror += error*update_rate;
                    last_error = error;

                    Eigen::Matrix<double,1,3> control = kp*error + ki*ierror + kd*derror;
                    // std::cout<<"P: "<<kp*error<<" I: "<<ki*ierror<<" D: "<<kd*derror<<std::endl;

                    std::vector<float> vel_float;
                    for(auto &vel : control) { 
                        if (abs(vel) > MAX_VELOCITY) vel = vel/abs(vel) * MAX_VELOCITY;
                        vel_float.push_back(static_cast<float>(vel));
                    }
                    vel_float.push_back(yaw_control);

                    // std::cout<<"vx: "<< control[0] <<std::endl;
                    // std::cout<<"vy: "<< control[1] <<std::endl;
                    // std::cout<<"vz: "<< control[2] <<std::endl;
                    // std::cout<<"yaw_control: "<< vel_float[3] <<std::endl;

                    std_msgs::msg::Float32MultiArray vel_msg = std_msgs::msg::Float32MultiArray();
                    vel_msg.data = vel_float;
                    velocity_pub->publish(vel_msg);

                }

            };
            this->declare_parameter<double>("update_rate",0.0);
            update_rate = this->get_parameter("update_rate").as_double();
            this->declare_parameter<double>("kp", 60*update_rate);
            this->declare_parameter<double>("ki", 1*update_rate);
            //initialize control parameters
            this->declare_parameter<double>("kd",30*update_rate);
            point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/current_point",2,point_callback);
            pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/quad_pose",2,pose_callback);
            velocity_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/velocities",2);

        }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto controller_node = std::make_shared<ControllerNode>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}