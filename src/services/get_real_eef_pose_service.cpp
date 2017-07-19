#include "../parameters.hpp"
#include "record_baxter_eef_trajectory/lib_recording.hpp"
#include "record_baxter_eef_trajectory/Getrealeefpose.h"
#include <boost/bind.hpp>

#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <sensor_msgs/PointCloud2.h>

// The parameters structure is used by all call backs, main and service
Data_config parameters;

//call back that register end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_left_eef_pose(l_eef_feedback, parameters);
}

//call back that register end effector pose and rearrange the orientation in RPY
void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_right_eef_pose(r_eef_feedback, parameters);
}

bool get_real_eef_pose_service_callback(record_baxter_eef_trajectory::Getrealeefpose::Request &req,
                                           record_baxter_eef_trajectory::Getrealeefpose::Response &res,
                                           ros::NodeHandle& nh,
                                           image_transport::ImageTransport& it_){

    //subscribers
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    usleep(1e6);

    if (req.eef_name == "left"){

        //get current eef pose
        Eigen::VectorXd current_values(6);
        current_values = parameters.get_left_eef_pose_rpy();

        res.pose = {current_values(0), current_values(1), current_values(2),
                    current_values(3), current_values(4), current_values(5)};
    } else {
        //get current eef pose
        Eigen::VectorXd current_values(6);
        current_values = parameters.get_right_eef_pose_rpy();

        res.pose = {current_values(0), current_values(1), current_values(2),
                    current_values(3), current_values(4), current_values(5)};
    }

    ROS_INFO("Done.");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_real_eef_pose_service");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  //int test;
  ros::ServiceServer service = n.advertiseService<record_baxter_eef_trajectory::Getrealeefpose::Request,
        record_baxter_eef_trajectory::Getrealeefpose::Response>("a2l/get_real_eef_pose",
                                                                    boost::bind(get_real_eef_pose_service_callback, _1, _2, n, it_));
  ROS_INFO("Ready to get real eef pose.");
  ros::spin();

  return 0;
}
