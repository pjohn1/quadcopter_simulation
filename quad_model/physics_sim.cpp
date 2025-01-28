#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "drone_properties.hpp"
#include <Eigen/Dense>
#include <cmath>

#define PI 3.1419

class PhysicsSim : public rclcpp::Node {
public:
    PhysicsSim() : Node("physics_sim"), x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {
        std::cout << "Physics Sim initialized" << std::endl;

        this->declare_parameter<double>("update_rate", 10.0);
        update_rate = this->get_parameter("update_rate").as_double();
        //Initialize Drone mass properties

        // Subscriber for forces and torques
        force_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/quadcopter/forces", 10,
            std::bind(&PhysicsSim::force_callback, this, std::placeholders::_1));

        pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("/quad_pose",1);

        // TF broadcaster, calculations are done in body frame so need to transform to map frame
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Timer for the simulation loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)round(1.0/update_rate*1000)),  // 10 Hz update rate
            std::bind(&PhysicsSim::update_pose, this));
    }

private:
    void force_callback(std_msgs::msg::Float32MultiArray msg) {
        std::cout<< "received wrench data" << std::endl;
        std::vector<float> vec = msg.data;
        vx = static_cast<double>(vec[0]);
        vy = static_cast<double>(vec[1]);
        vz = static_cast<double>(vec[2]);
        wx = static_cast<double>(vec[3]);
        wy = static_cast<double>(vec[4]);
        wz = static_cast<double>(vec[5]);

    }

    void update_pose() {

        // Publish the transform
        if (&roll != nullptr)
        {
            rclcpp::Time t = this->get_clock()->now();
            if (last_time.nanoseconds()!=0) 
            {   
                dt = t.seconds() - last_time.seconds();
                x = x + vx*dt;
                y = y + vy*dt;
                z = z + vz*dt;

                roll = std::fmod(roll + wx*dt, 2*PI);
                pitch = std::fmod(pitch + wy*dt, 2*PI);
                yaw = std::fmod(yaw + wz*dt, 2*PI);
            }

            last_time = t;
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->get_clock()->now();
            transform.header.frame_id = "map";  // Parent frame
            transform.child_frame_id = "base_link";  // Child frame

            // Position
            transform.transform.translation.x = x;
            transform.transform.translation.y = y;
            transform.transform.translation.z = z;

            // Orientation (convert yaw to quaternion)
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            // Broadcast the transform
            tf_broadcaster_->sendTransform(transform);

            geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;

            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            pose_pub->publish(pose);
        }
    }

    // Parameters
    
    // double mass_ = 1.0;          // Mass of the quadcopter
    // double inertia_z_ = 0.1;     // Moment of inertia for yaw
    // double dt_ = 0.01;           // Time step (100 Hz)
    rclcpp::Time last_time;
    double mass;
    double dt;
    double update_rate;

    // State variables
    double x, y, z = 0.0;           // Position
    double vx,vy,vz = 0.0;
    double wx,wy,wz = 0.0;
    double roll,pitch,yaw;

    // ROS 2 members
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PhysicsSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
