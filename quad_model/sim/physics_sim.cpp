#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "../headers/drone_properties.hpp"
#include <Eigen/Dense>
#include <cmath>

#define PI 3.1419

class PhysicsSim : public rclcpp::Node {
public:
    PhysicsSim() : Node("physics_sim"), x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {
        this->declare_parameter<double>("update_rate", 20.0);
        update_rate = this->get_parameter("update_rate").as_double();

        // Subscriber for forces and torques
        force_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/quadcopter/forces", 10,
            std::bind(&PhysicsSim::force_callback, this, std::placeholders::_1));

        pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/quad_pose",1);

        // TF broadcaster, calculations are done in body frame so need to transform to map frame
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Timer for the simulation loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)round(1.0/update_rate*1000)),  // 10 Hz update rate
            std::bind(&PhysicsSim::update_pose, this));
    }

private:
    void force_callback(std_msgs::msg::Float32MultiArray msg) {
        //simply sets the velocities received from phyiscs_node
        std::vector<float> vec = msg.data;
        vx = static_cast<double>(vec[0]);
        vy = static_cast<double>(vec[1]);
        vz = static_cast<double>(vec[2]);
        wx = static_cast<double>(vec[3]);
        wy = static_cast<double>(vec[4]);
        wz = static_cast<double>(vec[5]);

    }

    void update_pose() {

        if (&roll != nullptr)
        //ensure initialization has occurred
        {
            rclcpp::Time t = this->get_clock()->now();
            if (last_time.nanoseconds()!=0) 
            {   
                dt = t.seconds() - last_time.seconds();
                x = x + vx*dt;
                y = y + vy*dt;
                z = z + vz*dt;

                roll = std::fmod(roll + -wx*dt, 2*PI);
                // +y in transform frame seems to be to the right for some reason
                pitch = std::fmod(pitch + wy*dt, 2*PI);
                yaw = std::fmod(yaw + wz*dt, 2*PI);
            }

            last_time = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->get_clock()->now();
            transform.header.frame_id = "map";
            //transform between map(room) and base_link(drone)
            transform.child_frame_id = "base_link";

            transform.transform.translation.x = x;
            transform.transform.translation.y = y;
            transform.transform.translation.z = z;

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(transform);

            std_msgs::msg::Float32MultiArray pose_msg = std_msgs::msg::Float32MultiArray();
            //switched to multi array bc quaternion had issues w/ nonsingular values
            std::vector<double> dbl_data = {x,y,z,roll,pitch,yaw};
            std::vector<float> msg_data;
            //need to convert double vals to float for msg
            for(auto &vec : dbl_data) {msg_data.push_back(static_cast<float>(vec));}

            pose_msg.data = msg_data;
            pose_pub->publish(pose_msg);
        }
    }

    rclcpp::Time last_time;
    double mass;
    double dt;
    double update_rate;

    double x, y, z = 0.0;
    double vx,vy,vz = 0.0;
    double wx,wy,wz = 0.0;
    double roll,pitch,yaw;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub;
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
