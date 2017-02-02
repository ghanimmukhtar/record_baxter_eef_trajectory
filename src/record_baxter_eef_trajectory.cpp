#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <baxter_core_msgs/EndpointState.h>
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

void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    left_end_effector_pose.clear();
    left_eef_pose = l_eef_feedback.pose;
    tf::quaternionMsgToTF(left_eef_pose.orientation, my_rpy_orientation);
    double roll, yaw, pitch;
    tf::Matrix3x3 m(my_rpy_orientation);
    m.getRPY(roll, pitch, yaw);
    left_end_effector_pose.push_back(left_eef_pose.position.x);
    left_end_effector_pose.push_back(left_eef_pose.position.x);
    left_end_effector_pose.push_back(left_eef_pose.position.x);
    left_end_effector_pose.push_back(roll);
    left_end_effector_pose.push_back(pitch);
    left_end_effector_pose.push_back(yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_baxter_eef_trajectory");
    ros::NodeHandle node;
    ros::Subscriber sub_l_eef_msg = node.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 1, left_eef_Callback);
    double duration;
    node.getParam("duration", duration);
    ros::Duration my_duration(duration);
    std::ofstream file("./trajectorybackdrive");

    ros::AsyncSpinner spinner (1);
    spinner.start();


    std::vector<double> old_values = left_end_effector_pose;
    double norm, whole_time;
    if(file.is_open()){
        double begin = ros::Time::now().toSec();
        ROS_ERROR_STREAM("recording here ........., begin time is: " << begin);
        //ros::Rate rate(50.0);
        std::cout << "go go go" << std::endl;
        while ((ros::Time::now().toSec() - begin) < duration)
        {
            //whole_time = ros::Time::now().toSec() - begin;


            //get angles
            std::vector<double> current_values = left_end_effector_pose;
            norm = 0;
            for(int i = 0; i < old_values.size(); i++){
                norm = norm + (old_values[i] - current_values[i]) * (old_values[i] - current_values[i]);
            }
            if (current_values[0]*current_values[0] > 0.00001 && norm > 0.0005){
                for(int i = 0; i < current_values.size(); i++)
                    file << current_values[i] << " ";
                file << "\n";
                old_values = current_values;
            }
            //rate.sleep();
            ROS_ERROR_STREAM("recording here ........., ros time now is: " << ros::Time::now().toSec());
            ROS_WARN("******************************************************");
            ROS_ERROR_STREAM("recording here ........., begin time is: " << begin);
            ROS_ERROR_STREAM("output of the while condition is: " << ((ros::Time::now().toSec() - begin) < duration));
        }
    }
    else{
        std::cerr << "impossible to open the file" << std::endl;
        exit(1);
    }
    ROS_ERROR_STREAM("finished here ........., time now is: " << whole_time << " and duration is: " << duration);
    file.close();
    return 0;
}
