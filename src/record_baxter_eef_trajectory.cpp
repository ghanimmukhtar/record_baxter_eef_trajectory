/*#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

std::vector<double> left_joints_values(7);
void jocommCallback(sensor_msgs::JointState jo_state)
{
    left_joints_values[0] = jo_state.position[4];
    left_joints_values[1] = jo_state.position[5];
    left_joints_values[2] = jo_state.position[2];
    left_joints_values[3] = jo_state.position[3];
    left_joints_values[4] = jo_state.position[6];
    left_joints_values[5] = jo_state.position[7];
    left_joints_values[6] = jo_state.position[8];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle node;
    ros::Subscriber sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    double duration = atof(argv[1]);
    std::ofstream file("./trajectorybackdrive");
    ros::AsyncSpinner spinner (1);
    spinner.start();
    std::vector<double> old_values=left_joints_values;
    double norm;
    if(file.is_open()){
        double begin=ros::Time::now().toSec();
        ros::Rate rate(50.0);
        std::cout << "go go go" << std::endl;
        while (ros::Time::now().toSec()-begin < duration)
        {
            //get angles
            std::vector<double> current_values=left_joints_values;
            norm=0;
            for(int i = 0; i < 7; i++){
                norm=norm+(old_values[i]-current_values[i])*(old_values[i]-current_values[i]);
            }
            if (current_values[0]*current_values[0]>0.00001 && norm>0.0005){
                for(int i = 0; i < current_values.size(); i++)
                    file << current_values[i] << " ";
                file << "\n";
                old_values=current_values;
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
*/
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
    int number_paths = 0;
    if(file.is_open()){
        ros::Rate rate(50.0);
        rate.sleep();
        std::cout << "go go go" << std::endl;
        while (ros::ok())
        {
            if(pressed){
                if(release){
                    release = false;
                    toggle = true;
                }
                number_paths += 1;
                //get eef pose
                std::vector<double> current_values = left_end_effector_pose;
                norm = 0;
                for(int i = 0; i < old_values.size(); i++){
                    norm = norm + (old_values[i] - current_values[i]) * (old_values[i] - current_values[i]);
                }
                if (current_values[0]*current_values[0] > 0.00001 && norm > 0.0005){
                    for(int i = 0; i < current_values.size(); i++)
                        file << current_values[i] << ",";
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
