#ifndef __SIM_PARAM_HPP__
#define __SIM_PARAM_HPP__
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <Eigen/Core>

#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

#include <aruco/aruco.h>

#include <tf/tf.h>

struct Parameters {
    ///////// OBJECT tracking variables
    // object position (x, y, z) in real world in robot base frame
    cv_bridge::CvImagePtr cv_ptr;
    Eigen::Vector3d object_position;
    cv::Mat pic;
    sensor_msgs::ImageConstPtr rgb_msg, depth_msg;
    sensor_msgs::CameraInfoConstPtr info_msg;
    Eigen::Vector2i marker_center;

    aruco::MarkerDetector aruco_detector;
    aruco::CameraParameters camera_char;
    std::vector<aruco::Marker> markers;
    float marker_size = 0.04;

    std::string object_file_name = "";

    ////// ROBOT arm vairables
    // left end effector pose
    Eigen::VectorXd left_eef_pose_rpy;
    geometry_msgs::Pose left_eef_pose_quat;
    tf::Quaternion left_eef_rpy_orientation;

    std::string left_eef_trajectory_file_name = "";
    /////// SHARED variables
    int8_t pressed;
    bool release = true, toggle = false, l_lower_button_pressed = false;
    double epsilon = 0.01;
    std::string camera_param_path;
};

class Data_config{

public:
    Parameters params;

    //// Getters
    cv_bridge::CvImagePtr& get_cv_pridge(){
        return params.cv_ptr;
    }

    Eigen::Vector3d& get_object_position(){
        return params.object_position;
    }

    cv::Mat& get_cv_image(){
        return params.pic;
    }

    const sensor_msgs::ImageConstPtr& get_rgb_msg(){
        return params.rgb_msg;
    }

    const sensor_msgs::ImageConstPtr& get_depth_msg(){
        return params.depth_msg;
    }

    sensor_msgs::CameraInfoConstPtr& get_info_msg(){
        return params.info_msg;
    }

    Eigen::Vector2i& get_marker_center(){
        return params.marker_center;
    }

    aruco::MarkerDetector& get_aruco_detector(){
        return params.aruco_detector;
    }

    aruco::CameraParameters& get_camera_char(){
        return params.camera_char;
    }

    std::vector<aruco::Marker>& get_markers(){
        return params.markers;
    }

    float get_marker_size(){
        return params.marker_size;
    }

    std::string& get_object_file_name(){
        return params.object_file_name;
    }

    Eigen::VectorXd get_left_eef_pose_rpy(){
        return params.left_eef_pose_rpy;
    }

    geometry_msgs::Pose& get_left_eef_pose_quat(){
        return params.left_eef_pose_quat;
    }

    tf::Quaternion& get_left_eef_rpy_orientation(){
        return params.left_eef_rpy_orientation;
    }

    std::string& get_left_eef_trajectory_file_name(){
        return params.left_eef_trajectory_file_name;
    }

    int8_t& get_pressed(){
        return params.pressed;
    }

    bool& get_release(){
        return params.release;
    }

    bool& get_toggle(){
        return params.toggle;
    }

    bool& get_lower_botton_pressed(){
        return params.l_lower_button_pressed;
    }

    double& get_epsilon(){
        return params.epsilon;
    }

    std::string& get_camera_param_path(){
        return params.camera_param_path;
    }

    //// Setters
    void set_cv_pridged(cv_bridge::CvImagePtr& cv_ptr){
        params.cv_ptr = cv_ptr;
    }

    void set_object_position(Eigen::Vector3d& object_position){
        params.object_position = object_position;
    }

    void set_cv_image(cv::Mat& cv_image){
        params.pic = cv_image;
    }

    void set_rgb_msg(const sensor_msgs::ImageConstPtr& rgb_msg){
        params.rgb_msg = rgb_msg;
        //params.rgb_msg.reset(*rgb_msg);
    }

    void set_depth_msg(const sensor_msgs::ImageConstPtr& depth_msg){
        params.depth_msg = depth_msg;
        //params.depth_msg.reset(*depth_msg);
    }

    void set_info_msg(const sensor_msgs::CameraInfoConstPtr info_msg){
        params.info_msg = info_msg;
    }

    void set_marker_size(float& marker_size){
        params.marker_size = marker_size;
    }

    void set_marker_center(Eigen::Vector2i& marker_center){
        params.marker_center = marker_center;
    }

    void set_camera_char(aruco::CameraParameters& camera_char){
        params.camera_char = camera_char;
    }

    void set_markers(std::vector<aruco::Marker>& markers){
        params.markers = markers;
    }

    void set_object_file_name(std::string file_name){
        params.object_file_name = file_name;
    }

    void set_left_eef_pose_rpy(Eigen::VectorXd& left_eef_pose_rpy){
        params.left_eef_pose_rpy = left_eef_pose_rpy;
    }

    void set_left_eef_pose_quat(geometry_msgs::Pose& left_eef_pose_quat){
        params.left_eef_pose_quat = left_eef_pose_quat;
    }

    void set_left_eeft_rpy_orientation(tf::Quaternion& left_eef_rpy_orientation){
        params.left_eef_rpy_orientation = left_eef_rpy_orientation;
    }


    void set_left_eef_trajectory_file_name(std::string file_name){
        params.left_eef_trajectory_file_name = file_name;
    }

    void set_pressed(int8_t pressed){
        params.pressed = pressed;
    }

    void set_camera_param_path(std::string& camera_param_path){
        params.camera_param_path = camera_param_path;
    }

    void set_release(bool release){
        params.release = release;
    }

    void set_toggle(bool toggle){
        params.toggle = toggle;
    }

    void set_lower_button_pressed(bool l_lower_button_pressed){
        params.l_lower_button_pressed = l_lower_button_pressed;
    }

    void set_epsilon(double& epsilon){
        params.epsilon = epsilon;
    }
};

#endif
