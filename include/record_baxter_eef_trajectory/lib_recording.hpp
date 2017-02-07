#include "../../src/parameters.hpp"


#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <image_processing/DescriptorExtraction.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <pcl_conversions/pcl_conversions.h>

/*Prepare the cv image in the call back
 * input: the sensor image, and the Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void config_pic(const sensor_msgs::ImageConstPtr& msg, Data_config& parameters);

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

/*Recording method that uses all relevant parameters to control starting/stopping of recording
 * input: the Data_config class
 * return: nothing but it record the trajectory and object position in relevant files
 * */
void record_traj_and_object_position(Data_config& parameters, ofstream &left_eef_trajectory_file, ofstream &object_file);
