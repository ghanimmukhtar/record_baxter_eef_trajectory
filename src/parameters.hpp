#ifndef __SIM_PARAM_HPP__
#define __SIM_PARAM_HPP__
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <math.h>
#include <cmath>        // std::abs

#include <Eigen/Core>

#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <visual_functionalities/object_qr_position.h>
#include <visual_functionalities/object_blob_position.h>
#include <boost/bind.hpp>

#include <sensor_msgs/Image.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

struct Parameters {
    ///////// OBJECT tracking variables
    // object position (x, y, z) in real world in robot base frame
    size_t number_of_markers = 2;
    std::vector<Eigen::Vector3d> object_position_vector;

    std::map<int, std::vector<std::vector<double>>> objects_positions_map;
    std::vector<std::vector<double>> blob_positions;

    std::vector<int> markers_id_vector;

    std::string object_file_name = "";
    std::string child_frame, parent_frame, detection_method;

    ////// ROBOT arm vairables
    // left end effector pose
    Eigen::VectorXd left_eef_pose_rpy;
    geometry_msgs::Pose left_eef_pose_quat;
    tf::Quaternion left_eef_rpy_orientation;

    std::string left_eef_trajectory_file_name = "";
    /////// SHARED variables
    int8_t pressed;
    int64_t start_recording = 0;
    bool release = true, toggle = false, l_lower_button_pressed = false;
    double epsilon;
    double the_rate;
};

class Data_config{

public:
    Parameters params;

    Data_config(){
        //size setters
        params.object_position_vector.resize(params.number_of_markers);
        params.markers_id_vector.resize(params.number_of_markers);
    }

    void resize_vectors(int new_size){
        params.markers_id_vector.resize(new_size);
    }

    //// Getters
    int get_number_of_markers(){
        return params.number_of_markers;
    }

    std::vector<std::vector<double>>& get_blob_positions(){
                                    return params.blob_positions;
}

    std::map<int, std::vector<std::vector<double>>>& get_objects_positions_map(){
                                                  return params.objects_positions_map;
}

    Eigen::Vector3d& get_object_position(int index){
        return params.object_position_vector[index];
    }

    std::vector<Eigen::Vector3d>& get_object_position_vector(){
        return params.object_position_vector;
    }

    std::vector<int>& get_markers_id_vector(){
        return params.markers_id_vector;
    }

    std::string& get_object_file_name(){
        return params.object_file_name;
    }

                                                  std::string& get_detection_method(){
                                                      return params.detection_method;
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

                                                  std::string& get_child_frame(){
                                                      return params.child_frame;
                                                  }

                                                  std::string& get_parent_frame(){
                                                      return params.parent_frame;
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

                                                  int64_t& get_start_recording(){
                                                  return params.start_recording;
}

    double& get_epsilon(){
        return params.epsilon;
    }

    double& get_the_rate(){
        return params.the_rate;
    }

    //// Setters
    void set_number_of_marker(int number){
        params.number_of_markers = number;
    }

                                                  void set_blob_positions(std::vector<std::vector<double>> positions){
                                                  params.blob_positions = positions;
}

    void set_objects_positions_map(std::map<int, std::vector<std::vector<double>>> map){
         params.objects_positions_map = map;
}

    void set_object_position(Eigen::Vector3d& object_position, int index){
        params.object_position_vector[index] = object_position;
    }

    void set_markers_id_vector(int marker_id, int marker_index){
        params.markers_id_vector[marker_index] = marker_id;
    }

    void set_object_file_name(std::string file_name){
        params.object_file_name = file_name;
    }

                                                  void set_detection_method(std::string detection_method){
                                                      params.detection_method = detection_method;
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
                                                  void set_child_frame(std::string frame){
                                                      params.child_frame = frame;
                                                  }

                                                  void set_parent_frame(std::string frame){
                                                      params.parent_frame = frame;
                                                  }

                                                  void set_start_recording(int64_t start){
                                                  params.start_recording = start;
}
};

#endif
