#include "../parameters.hpp"
#include "record_baxter_eef_trajectory/lib_recording.hpp"
#include "record_baxter_eef_trajectory/ObjectPosition.h"
#include <boost/bind.hpp>

#include <sensor_msgs/image_encodings.h>
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

bool object_position_feedback(record_baxter_eef_trajectory::ObjectPosition::Request &req,
                          record_baxter_eef_trajectory::ObjectPosition::Response &res,
                          ros::NodeHandle& nh,
                                   image_transport::ImageTransport& it_){


    //nh.getParam("camera_param_path", parameters.get_camera_param_path());
    parameters.get_camera_char().readFromXMLFile("/home/ghanim/git/automatic_camera_robot_cal/data/camera_param_baxter.xml");

    //subscribers
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber in_info_image = nh.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCb);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

        res.x = parameters.get_object_position().;


    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_position_feedback");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  //int test;
  ros::ServiceServer service = n.advertiseService<record_baxter_eef_trajectory::ObjectPosition::Request,
        record_baxter_eef_trajectory::ObjectPosition::Response>("baxter_left_trajectory/obj_position_srv",
                                                            boost::bind(object_position_feedback, _1, _2, n, it_));
  ROS_INFO("Ready to feedback object position.");
  ros::spin();

  return 0;
}
