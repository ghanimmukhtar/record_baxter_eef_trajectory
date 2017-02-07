#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

std::vector<double> left_end_effector_pose(6);
geometry_msgs::Pose left_eef_pose;
tf::Quaternion my_rpy_orientation;
int8_t pressed;

//call back that register end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    left_end_effector_pose.clear();
    left_eef_pose = l_eef_feedback.pose;
    tf::quaternionMsgToTF(left_eef_pose.orientation, my_rpy_orientation);
    double roll, yaw, pitch;
    tf::Matrix3x3 m(my_rpy_orientation);
    m.getRPY(roll, pitch, yaw);
    left_end_effector_pose.push_back(left_eef_pose.position.x);
    left_end_effector_pose.push_back(left_eef_pose.position.y);
    left_end_effector_pose.push_back(left_eef_pose.position.z);
    left_end_effector_pose.push_back(roll);
    left_end_effector_pose.push_back(pitch);
    left_end_effector_pose.push_back(yaw);
}

//call back to monitor the status of the cuff of the left gripper
void left_cuf_Callback(baxter_core_msgs::DigitalIOState l_cuf_feedbcak){
    pressed = l_cuf_feedbcak.state;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_baxter_eef_trajectory");
    ros::NodeHandle node;
    ros::Subscriber sub_l_eef_msg = node.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_l_cuf_msg = node.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 10, left_cuf_Callback);
    //node.getParam("duration", duration);
    std::ofstream file("./trajectorybackdrive");

    ros::AsyncSpinner spinner (1);
    spinner.start();

    std::vector<double> old_values = left_end_effector_pose;
    double norm;
    pressed = false;
    bool release = true, toggle = false;
    double epsilon = 0.01;
    if(file.is_open()){
        ros::Rate rate(50.0);
        rate.sleep();
        ROS_INFO("go go go");
        while (ros::ok())
        {
            if(pressed){
                if(release){
                    release = false;
                    toggle = true;
                }
                //get eef pose
                std::vector<double> current_values = left_end_effector_pose;
                norm = 0;
                std::vector<double>::iterator itr;
                int j = 0;
                for(itr = old_values.begin(); itr < old_values.end(); ++itr){
                    norm = norm + (*itr - current_values[j]) * (*itr - current_values[j]);
                    j += 1;
                }
                if (norm > epsilon){
                    for(itr = current_values.begin(); itr < current_values.end(); ++itr)
                        file << (*itr) << ",";
                    old_values = current_values;
                }
            }
            else
                release = true;
            if(toggle && !pressed){
                file << "\n";
                toggle = false;
            }
            rate.sleep();
        }
    }
    else{
        std::cerr << "impossible to open the file" << std::endl;
        exit(1);
    }
    file.close();
    return 0;
}
