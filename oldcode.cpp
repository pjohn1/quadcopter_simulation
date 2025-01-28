auto pose_callback = [this](const geometry_msgs::msg::Pose &msg) -> void
            {
                if (initialized)
                {
                    x = msg.position.x;
                    y = msg.position.y;
                    z = msg.position.z;
                    Eigen::Matrix<double,1,3> pose(x,y,z);
                    std::vector<double> voltages = {0.0,0.0,0.0,0.0};

                    Eigen::Quaterniond q2(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
                    Eigen::Vector3d euler = q2.toRotationMatrix().eulerAngles(2,1,0);
                    yaw = euler[2];pitch=euler[1];roll=euler[0];

                    Eigen::Matrix<double,1,3> pose_difference = goal_pose-pose;
                    double desired_pitch = 30 * PI/180; //desired forward pitch
                    double yaw_difference = angle_difference(goal_yaw,yaw);
                    double pitch_difference = desired_pitch - pitch;

                    double P_yaw = kp_yaw*yaw_difference;


                    if (abs(yaw_difference) > 0.1 && yaw_difference > 0.0)
                    {
                        voltages[0] = P_yaw;
                        voltages[2] = P_yaw;
                    }

                    else if (abs(yaw_difference) > 0.1 && yaw_difference < 0.0)
                    {
                        voltages[1] = P_yaw;
                        voltages[3] = P_yaw;
                    }

                    else if (abs(pose_difference[0]) > 0.1 && z < 0.5)
                    {
                        for(auto &voltage : voltages) {voltage=0.2;}
                    }

                    else if (abs(pose_difference[0]) > 0.1)
                    {
                        if (pitch_difference > 0.0)
                        {
                            voltages[2] = kp*pitch_difference;
                            voltages[3] = kp*pitch_difference;
                        }
                        else {
                            voltages[0] = -kp*pitch_difference;
                            voltages[1] = -kp*pitch_difference;
                        }
                    }

                    // else if (abs(pose_difference[0]) > 0.1)
                    // {
                    //     voltages[2] = P[0];
                    //     voltages[3] = P[0];
                    // }

                    // else if (pose_difference[2] < -0.1) {
                    //     for(auto &voltage : voltages) {voltage+=P[2];}
                    // }
                    
                    
                    std::vector<float> voltages_float;
                    std::cout<<"goal yaw: "<<goal_yaw<<" current: " << yaw <<std::endl;
                    for(auto &voltage : voltages) {voltages_float.push_back(static_cast<float>(voltage));std::cout<<voltage<<std::endl;}
                    std::cout<<std::endl;

                    std_msgs::msg::Float32MultiArray voltage_msg = std_msgs::msg::Float32MultiArray();
                    voltage_msg.data = voltages_float;
                    voltage_pub->publish(voltage_msg);
