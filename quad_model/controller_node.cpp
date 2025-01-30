#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.1419

class ControllerNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub;

        Eigen::Matrix<double,1,3> goal_pose;
        double x,y,z = 0.0;
        double yaw,pitch,roll=0.0;
        double goal_yaw = 0.0;
        bool initialized = false;

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
                // goal_pose << 1.0,0.0,1.0;
                goal_yaw = std::atan2(msg.point.y,msg.point.x);
                std::cout << goal_yaw << std::endl;
                initialized = true;
            };

            auto pose_callback = [this](const geometry_msgs::msg::Pose &msg) -> void
            {
                if (initialized)
                {
                    x = msg.position.x;
                    y = msg.position.y;
                    z = msg.position.z;
                    Eigen::Matrix<double,1,3> pose(x,y,z);
                    std::vector<double> velocities = {0.0,0.0,0.0,0.0};
                    kp = this->get_parameter("kp").as_double();
                    //vx,vy,vz,yaw rate (CCW)

                    Eigen::Quaterniond q2(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
                    Eigen::Vector3d euler = q2.toRotationMatrix().eulerAngles(2,1,0);
                    yaw = euler[2];pitch=euler[1];roll=euler[0];

                    Eigen::Matrix<double,1,3> pose_difference = goal_pose-pose;
                    std::cout<<std::endl;
                    std::cout<<"pose difference: "<<pose_difference<<std::endl;
                    double yaw_difference = angle_difference(goal_yaw,yaw);
                    
                    std::cout<<"kp:"<< kp <<std::endl;
                    Eigen::Matrix<double,1,3> control = kp*pose_difference;

                    float yaw_control = kp_yaw * static_cast<float>(yaw_difference);

                    std::vector<float> vel_float;
                    for(auto &vel : control) { vel_float.push_back(static_cast<float>(vel));}
                    vel_float.push_back(yaw_control);

                    std::cout<<"vx: "<< control[0] <<std::endl;
                    std::cout<<"vy: "<< control[1] <<std::endl;
                    std::cout<<"vz: "<< control[2] <<std::endl;
                    std::cout<<"yaw_control: "<< vel_float[3] <<std::endl;


                    std_msgs::msg::Float32MultiArray vel_msg = std_msgs::msg::Float32MultiArray();
                    vel_msg.data = vel_float;
                    velocity_pub->publish(vel_msg);

                }

            };
            this->declare_parameter<double>("kp", 0.1);
            point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point",1,point_callback);
            pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("/quad_pose",1,pose_callback);
            velocity_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/velocities",1);

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