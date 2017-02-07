#include "../parameters.hpp"
#include "record_baxter_eef_trajectory/lib_recording.hpp"
#include "record_baxter_eef_trajectory/Recordtraj.h"
#include <boost/bind.hpp>

#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <sensor_msgs/PointCloud2.h>

// The parameters structure is used by all call backs, main and service
Data_config parameters;

//get the pic variable filled, and show a circle and a rectangle around the marker
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    config_pic(msg, parameters);
    config_and_detect_markers(parameters);
    show_marker(parameters);
}

//get camera info to make the tracking accurate
void infoimageCb(const sensor_msgs::CameraInfoConstPtr msg){
    parameters.set_info_msg(msg);
}

//get object position (x, y and z)
void depthimageCb(const sensor_msgs::ImageConstPtr& depth_msg){
    locate_object(depth_msg, parameters);
}

//call back that register end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_left_eef_pose(l_eef_feedback, parameters);
}

//call back to monitor the status of the cuff of the left gripper
void left_cuf_Callback(baxter_core_msgs::DigitalIOState l_cuf_feedbcak){
    parameters.set_pressed(l_cuf_feedbcak.state);
}

//call back to monitor the status of the lower button of the left gripper
void left_lower_button_Callback(baxter_core_msgs::DigitalIOState l_lower_button_feedbcak){
    if(l_lower_button_feedbcak.state)
        parameters.set_lower_button_pressed(true);
}

bool left_eef_trajectory_recording(record_baxter_eef_trajectory::Recordtraj::Request &req,
                          record_baxter_eef_trajectory::Recordtraj::Response &res,
                          ros::NodeHandle& nh,
                                   image_transport::ImageTransport& it_){


    //nh.getParam("camera_param_path", parameters.get_camera_param_path());
    parameters.get_camera_char().readFromXMLFile("/home/ghanim/git/automatic_camera_robot_cal/data/camera_param_baxter.xml");

    //subscribers
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber in_info_image = nh.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCb);
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_l_cuf_msg = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 10, left_cuf_Callback);
    ros::Subscriber sub_l_lower_button = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_button/state", 10, left_lower_button_Callback);
    ros::Publisher image_publisher = nh.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);

    parameters.set_object_file_name("./object_positions");
    parameters.set_left_eef_trajectory_file_name("./eef_trajectory_recorder");

    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();
    usleep(1e6);

    std::ofstream object_file("object_positions");
    std::ofstream left_eef_trajectory_file;
    left_eef_trajectory_file.open("/home/ghanim/git/catkin_ws/eef_trajectory_recorder.csv");

    ros::Rate rate(50.0);
    rate.sleep();
    while(ros::ok() && req.start){
        record_traj_and_object_position(parameters, left_eef_trajectory_file, object_file, image_publisher);
        rate.sleep();
    }
    left_eef_trajectory_file.close();
    object_file.close();
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_left_eef_trajectory");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  //int test;
  ros::ServiceServer service = n.advertiseService<record_baxter_eef_trajectory::Recordtraj::Request,
        record_baxter_eef_trajectory::Recordtraj::Response>("baxter_left_trajectory/recording_srv",
                                                            boost::bind(left_eef_trajectory_recording, _1, _2, n, it_));
  ROS_INFO("Ready to record baxter left eef trajectories and object positions.");
  ros::spin();

  return 0;
}
