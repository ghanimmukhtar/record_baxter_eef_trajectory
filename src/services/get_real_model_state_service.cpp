#include "../parameters.hpp"
#include "record_baxter_eef_trajectory/lib_recording.hpp"
#include "record_baxter_eef_trajectory/Getrealmodelstate.h"
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
    //show_marker(parameters);
}

//get camera info to make the tracking accurate
void infoimageCb(const sensor_msgs::CameraInfoConstPtr msg){
    parameters.set_info_msg(msg);
}

//get object position (x, y and z)
void depthimageCb(const sensor_msgs::ImageConstPtr& depth_msg){
    locate_object(depth_msg, parameters);
}

bool get_real_model_state_service_callback(record_baxter_eef_trajectory::Getrealmodelstate::Request &req,
                                           record_baxter_eef_trajectory::Getrealmodelstate::Response &res,
                                           ros::NodeHandle& nh,
                                           image_transport::ImageTransport& it_){

    parameters.get_camera_char().readFromXMLFile("/home/mukhtar/git/automatic_camera_robot_cal/data/camera_param_baxter.xml");

    //subscribers
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber in_info_image = nh.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCb);
//    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
//    ros::Subscriber sub_l_cuf_msg = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 10, left_cuf_Callback);
//    ros::Subscriber sub_l_lower_button = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_button/state", 10, left_lower_button_Callback);
    ros::Publisher image_publisher = nh.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    usleep(1e6);

//    std::ofstream object_file("object_positions");
//    std::ofstream left_eef_trajectory_file;
//    left_eef_trajectory_file.open("/home/ghanim/git/catkin_ws/eef_trajectory_recorder.csv");
//    std::vector<std::vector<double>> outputs;

//    ros::Rate rate(50.0);
//    rate.sleep();
//    while(ros::ok() && req.start){
//        record_traj_and_object_position(parameters, outputs, image_publisher, left_eef_trajectory_file);
//        rate.sleep();
//    }
//    left_eef_trajectory_file.close();
//    object_file.close();
//    return true;

    ros::Rate rate(50.0);
    rate.sleep();

    usleep(4e6);

    Eigen::Vector3d obj_pos = parameters.get_object_position();
    Eigen::Vector4d extended_vector;
    extended_vector << obj_pos[0],
                       obj_pos[1],
                       obj_pos[2],
                       1;
    std::vector<Eigen::Vector4d> vector;
    vector.push_back(extended_vector);
    std::vector<std::vector<double>> output_of_conversion;
    convert_whole_object_positions_vector(vector, output_of_conversion);
    std::vector<double> obj_pos_converted = output_of_conversion[0];

    res.model_state = {obj_pos_converted[0],
                       obj_pos_converted[1],
                       obj_pos_converted[2],
                       0, 0, 0};

    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_real_model_state_service");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  //int test;
  ros::ServiceServer service = n.advertiseService<record_baxter_eef_trajectory::Getrealmodelstate::Request,
        record_baxter_eef_trajectory::Getrealmodelstate::Response>("a2l/get_model_state",
                                                                    boost::bind(get_real_model_state_service_callback, _1, _2, n, it_));
  ROS_INFO("Ready to get real object state.");
  ros::spin();

  return 0;
}
