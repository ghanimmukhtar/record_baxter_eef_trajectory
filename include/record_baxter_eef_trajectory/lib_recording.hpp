#include "../../src/parameters.hpp"

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <image_processing/DescriptorExtraction.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <cv_bridge/cv_bridge.h>

/*Prepare the cv image in the call back
 * input: the sensor image, and the Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void config_pic(const sensor_msgs::ImageConstPtr& msg, Data_config& parameters);

void get_angles_from_rotation_matrix(tf::Matrix3x3& rotation_matrix, std::vector<double>& returned_angles, std::string my_type = "quaternion");

void set_camera_poses_and_transformation(Data_config& parameters);

/*Configure the markers detector as required
 * input: Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void config_and_detect_markers(Data_config& parameters);

/*Show the marker on an image window
 * input: Data_config class
 * return: nothing but it show the marker and a circle around the point it follows
 * */
void show_marker(Data_config& parameters);

/*get object position
 * input: the depth image and the Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_object(const sensor_msgs::ImageConstPtr& depth_msg, Data_config& parameters);

/*get baxter left eef pose with orientation expressed as RPY
 * input: a baxter core msg that holds left eef status (including the pose as geometry msgs), and the Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_left_eef_pose(baxter_core_msgs::EndpointState& l_eef_feedback, Data_config& parameters);

/*Convert object position from camera frame to robot frame
 * input: object position in camera frame, and a vector to hold the transformation into robot frame
 * return: nothing but it record the trajectory and object position in relevant files
 * */
void convert_object_position_to_robot_base(Eigen::Vector3d& object_pose_in_camera_frame, Eigen::Vector3d& object_pose_in_robot_frame);

/*Display some images on baxter screen to reflect what he is doing
 * input: the path to the image to be displayed
 * return: nothing but just display the image
 * */
void dispaly_image(std::string path, ros::Publisher &image_pub);

void convert_whole_object_positions_vector(Data_config &parameters,
                                           std::vector<Eigen::Vector4d>& object_positions_vector,
                                           std::vector<std::vector<double> > &output_of_conversion);
void write_data(std::vector<std::vector<double>>& left_eef_trajectory_and_object_vector,
                std::vector<std::vector<double>>& object_positions_vector, std::ofstream& the_file);

/*Recording method that uses all relevant parameters to control starting/stopping of recording
 * input: the Data_config class
 * return: nothing but it record the trajectory and object position in relevant files
 * */
void record_traj_and_object_position(Data_config& parameters, std::vector<std::vector<double> > &left_eef_trajectory, ros::Publisher &image_pub, ofstream &the_file);
