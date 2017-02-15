#include "parameters.hpp"
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_left_eef_trajectory");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  //nh.getParam("camera_param_path", parameters.get_camera_param_path());
  parameters.get_camera_char().readFromXMLFile("/home/mukhtar/git/record_baxter_eef_trajectory/data/camera_param_baxter.xml");

  //subscribers
  image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
  ros::Subscriber in_info_image = n.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, infoimageCb);
  image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCb);
  ros::Subscriber sub_l_eef_msg = n.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
  ros::Subscriber sub_l_cuf_msg = n.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 10, left_cuf_Callback);
  ros::Subscriber sub_l_lower_button = n.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_button/state", 10, left_lower_button_Callback);
  ros::Publisher image_publisher = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);

  parameters.set_object_file_name("./object_positions");
  parameters.set_left_eef_trajectory_file_name("./eef_trajectory_recorder");

  ros::AsyncSpinner my_spinner(4);
  my_spinner.start();
  usleep(1e6);

  std::ofstream object_file("object_positions.csv");
  std::ofstream left_eef_trajectory_file("eef_trajectory_recorder.csv");


  record_traj_and_object_position(parameters, left_eef_trajectory_file, object_file, image_publisher);

  left_eef_trajectory_file.close();
  object_file.close();

  return 0;
}


/*
//subscribe to rgb input and look in the incoming stream for a a certain object (baxter gripper)
#include <ros/ros.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>

#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <image_processing/DescriptorExtraction.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace cv;

//eef trajectory recorder variables
std::vector<double> left_end_effector_pose(6);
geometry_msgs::Pose left_eef_pose;
tf::Quaternion my_rpy_orientation;

//object tracking variables
Mat pic;
sensor_msgs::ImageConstPtr rgb_msg;
sensor_msgs::CameraInfoConstPtr info_msg;
float my_x,my_y;
double xc = 0.0, yc = 0.0, zc = 0.0;
aruco::CameraParameters camera_char;
vector<aruco::Marker> markers;

//shared variables
int8_t pressed;

//get the pic variable filled, and show a circle and a rectangle around the marker
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    rgb_msg = msg;
    namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    pic = cv_ptr->image;

    aruco::MarkerDetector my_detector;
    my_detector.setDictionary("ARUCO");
    my_detector.detect(pic,markers,camera_char,0.04);
    if (!markers.empty()){
        //ROS_WARN_STREAM("marker size is: " << markers[0].size());
        markers[0].draw(pic,cv::Scalar(94.0, 206.0, 165.0, 0.0));
        markers[0].calculateExtrinsics(0.04,camera_char,false);
        my_x = (int) (markers[0][0].x + markers[0][2].x)/2;
        my_y = (int) (markers[0][0].y + markers[0][2].y)/2;
        circle(pic, cv::Point((markers[0][0].x + markers[0][2].x)/2, (markers[0][0].y + markers[0][2].y)/2), 10, CV_RGB(255,0,0));
    }
    imshow("ShowMarker", pic);
    waitKey(1);
}

//get camera info to make the tracking accurate
void infoimageCb(const sensor_msgs::CameraInfoConstPtr msg){
    info_msg = msg;
}

//get the position (x, y and z)
void depthimageCb(const sensor_msgs::ImageConstPtr& depth_msg){
    if(!pic.empty() && !markers.empty()){
        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);
        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
        image_processing::PointCloudT::Ptr input_cloud(new image_processing::PointCloudT);

        pcl::fromROSMsg(ptcl_msg, *input_cloud);
        image_processing::PointT pt = input_cloud->at((int) my_x+(int) my_y*input_cloud->width);
        zc = pt.z;
        xc = pt.x;
        yc = pt.y;
    }
}

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
    ros::init(argc, argv, "record_object_position");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    camera_char.readFromXMLFile("/home/mukhtar/git/catkin_ws/src/automatic_camera_robot_cal/data/camera_param_baxter.xml");
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber in_info_image = node.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCb);
    ros::Subscriber sub_l_eef_msg = node.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_l_cuf_msg = node.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 10, left_cuf_Callback);

    std::ofstream object_file("./object_positions");
    std::ofstream eef_trajectory_file("./eef_trajectory_recorder");

    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();

    //record marker position (object position) if changed in the specified file
    std::vector<double> old_values = left_end_effector_pose;
    double norm;
    pressed = false;
    bool release = true, toggle = false;
    double epsilon = 0.01;
    if(object_file.is_open()){
        ros::Rate rate(50.0);
        rate.sleep();
        std::cout << "go go go" << std::endl;
        while (ros::ok()){
            if(pressed){
                if(release){
                    release = false;
                    toggle = true;
                }
                while(xc != xc || yc != yc || zc != zc);
                //get current eef pose
                std::vector<double> current_values = left_end_effector_pose;
                std::vector<double>::iterator itr;
                int j = 0;
                for(itr = old_values.begin(); itr < old_values.end(); ++itr){
                    norm = norm + (*itr - current_values[j]) * (*itr - current_values[j]);
                    j += 1;
                }
                if (norm > epsilon){
                    for(itr = current_values.begin(); itr < current_values.end(); ++itr)
                        eef_trajectory_file << (*itr) << ",";
                    old_values = current_values;
                    //while saving end effector changes register object position concurrently
                    object_file << xc << "," << yc << "," << zc;
                }

            }
            else
                release = true;
            if(toggle && !pressed){
                eef_trajectory_file << "\n";
                object_file << "\n";
                toggle = false;
            }
            rate.sleep();
        }
    }
    return 0;
}
*/
