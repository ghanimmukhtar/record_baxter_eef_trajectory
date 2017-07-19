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
    size_t number_of_markers = 1;
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<Eigen::Vector3d> object_position_vector;


    cv::Mat pic;
    sensor_msgs::ImageConstPtr rgb_msg, depth_msg;
    sensor_msgs::CameraInfoConstPtr info_msg;
    std::vector<Eigen::Vector2i> marker_center_vector;


    aruco::MarkerDetector aruco_detector;
    aruco::CameraParameters camera_char;
    std::vector<aruco::Marker> markers_vector;
    std::vector<int> markers_id_vector;

    float marker_size = 0.05;

    std::string object_file_name = "";

    ////// ROBOT arm vairables
    // left end effector pose
    Eigen::VectorXd left_eef_pose_rpy;
    geometry_msgs::Pose left_eef_pose_quat;
    tf::Quaternion left_eef_rpy_orientation;
    // right end effector pose
    Eigen::VectorXd right_eef_pose_rpy;
    geometry_msgs::Pose right_eef_pose_quat;
    tf::Quaternion right_eef_rpy_orientation;

    std::string left_eef_trajectory_file_name = "";
    /////// SHARED variables
    int8_t pressed;
    bool release = true, toggle = false, l_lower_button_pressed = false;
    double epsilon = 0.01;
    std::string camera_param_path;
    double the_rate = 50.0;
    std::vector<double> camera_rgb_optical_pose, camera_depth_optical_pose, camera_link_pose;
    Eigen::Matrix4d Trans_M;
    std::string camera_fram_pose;
    std::string camera_frame_choice;
};

class Data_config{

public:
    Parameters params;

    Data_config(){
        //size setters
        params.object_position_vector.resize(params.number_of_markers);
        params.marker_center_vector.resize(params.number_of_markers);
        params.markers_vector.resize(params.number_of_markers);
        params.markers_id_vector.resize(params.number_of_markers);
    }

    void resize_vectors(int new_size){
        params.markers_id_vector.resize(new_size);
    }

    //// Getters
    int get_number_of_markers(){
        return params.number_of_markers;
    }

    cv_bridge::CvImagePtr& get_cv_pridge(){
        return params.cv_ptr;
    }

    Eigen::Vector3d& get_object_position(int index){
        return params.object_position_vector[index];
    }

    std::vector<Eigen::Vector3d>& get_object_position_vector(){
        return params.object_position_vector;
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

    Eigen::Vector2i& get_marker_center(int index){
        return params.marker_center_vector[index];
    }

    std::vector<Eigen::Vector2i>& get_marker_center_vector(){
        return params.marker_center_vector;
    }

    aruco::MarkerDetector& get_aruco_detector(){
        return params.aruco_detector;
    }

    aruco::CameraParameters& get_camera_char(){
        return params.camera_char;
    }

    std::vector<aruco::Marker>& get_markers(){
        return params.markers_vector;
    }

    std::vector<int>& get_markers_id_vector(){
        return params.markers_id_vector;
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

    Eigen::VectorXd get_right_eef_pose_rpy(){
        return params.right_eef_pose_rpy;
    }

    geometry_msgs::Pose& get_left_eef_pose_quat(){
        return params.left_eef_pose_quat;
    }

    geometry_msgs::Pose& get_right_eef_pose_quat(){
        return params.right_eef_pose_quat;
    }

    tf::Quaternion& get_left_eef_rpy_orientation(){
        return params.left_eef_rpy_orientation;
    }

    tf::Quaternion& get_right_eef_rpy_orientation(){
        return params.right_eef_rpy_orientation;
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

    double& get_the_rate(){
        return params.the_rate;
    }

    std::string& get_camera_param_path(){
        return params.camera_param_path;
    }

    std::vector<double>& get_camera_depth_optical_pose(){
        return params.camera_depth_optical_pose;
    }

    std::vector<double>& get_camera_rgb_optical_pose(){
        return params.camera_rgb_optical_pose;
    }

    std::vector<double>& get_camera_link_pose(){
        return params.camera_link_pose;
    }

    Eigen::Matrix4d& get_transformation_matrix(){
        return params.Trans_M ;
    }

    std::string& get_camera_frame_pose(){
        return params.camera_fram_pose;
    }

    std::string& get_camera_frame_choice(){
        return params.camera_frame_choice;
    }

    //// Setters
    void set_number_of_marker(int number){
        params.number_of_markers = number;
    }

    void set_cv_pridged(cv_bridge::CvImagePtr& cv_ptr){
        params.cv_ptr = cv_ptr;
    }

    void set_object_position(Eigen::Vector3d& object_position, int index){
        params.object_position_vector[index] = object_position;
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

    void set_marker_center(Eigen::Vector2i& marker_center, int index){
        params.marker_center_vector[index] = marker_center;
    }

    void set_camera_char(aruco::CameraParameters& camera_char){
        params.camera_char = camera_char;
    }

    void set_markers_id_vector(int marker_id, int marker_index){
        params.markers_id_vector[marker_index] = marker_id;
    }

    void set_markers(std::vector<aruco::Marker>& markers){
        params.markers_vector = markers;
    }

    void set_object_file_name(std::string file_name){
        params.object_file_name = file_name;
    }

    void set_left_eef_pose_rpy(Eigen::VectorXd& left_eef_pose_rpy){
        params.left_eef_pose_rpy = left_eef_pose_rpy;
    }

    void set_right_eef_pose_rpy(Eigen::VectorXd& right_eef_pose_rpy){
        params.right_eef_pose_rpy = right_eef_pose_rpy;
    }

    void set_left_eef_pose_quat(geometry_msgs::Pose& left_eef_pose_quat){
        params.left_eef_pose_quat = left_eef_pose_quat;
    }

    void set_right_eef_pose_quat(geometry_msgs::Pose& right_eef_pose_quat){
        params.right_eef_pose_quat = right_eef_pose_quat;
    }

    void set_left_eef_rpy_orientation(tf::Quaternion& left_eef_rpy_orientation){
        params.left_eef_rpy_orientation = left_eef_rpy_orientation;
    }

    void set_right_eef_rpy_orientation(tf::Quaternion& right_eef_rpy_orientation){
        params.right_eef_rpy_orientation = right_eef_rpy_orientation;
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

    void set_the_rate(double& the_rate){
        params.the_rate = the_rate;
    }

    void set_camera_depth_optical_pose(std::vector<double>& pose){
        params.camera_depth_optical_pose = pose;
    }

    void set_camera_rgb_optical_pose(std::vector<double>& pose){
        params.camera_rgb_optical_pose = pose;
    }

    void set_camera_link_pose(std::vector<double>& pose){
        params.camera_link_pose = pose;
    }

    void set_transformation_matrix(Eigen::Matrix4d& Trans_M){
        params.Trans_M = Trans_M;
    }

    void set_camera_frame_pose(std::string pose){
        params.camera_fram_pose = pose;
    }

    void set_camera_frame_choice(std::string frame_choice){
        params.camera_frame_choice = frame_choice;
    }
};

#endif
