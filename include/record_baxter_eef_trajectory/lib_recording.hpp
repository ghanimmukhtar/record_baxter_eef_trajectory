#include "../../src/parameters.hpp"

void get_angles_from_rotation_matrix(tf::Matrix3x3& rotation_matrix,
                                     std::vector<double>& returned_angles,
                                     std::string my_type = "quaternion");

void record_qr_position(const visual_functionalities::object_qr_positionConstPtr& object_position,
                            Data_config &parameters);

void record_blob_position(const visual_functionalities::object_blob_positionConstPtr& object_position,
                            Data_config &parameters);

/*get baxter left eef pose with orientation expressed as RPY
 * input: a baxter core msg that holds left eef status (including the pose as geometry msgs), and the Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_left_eef_pose(baxter_core_msgs::EndpointState& l_eef_feedback, Data_config& parameters);

/*Display some images on baxter screen to reflect what he is doing
 * input: the path to the image to be displayed
 * return: nothing but just display the image
 * */
void dispaly_image(std::string path, ros::Publisher &image_pub);

void convert_whole_object_positions_vector(Data_config &parameters,
                                           std::vector<Eigen::Vector4d>& object_positions_vector,
                                           std::vector<std::vector<double> > &output_of_conversion);

void write_data(Data_config& parameters,
                std::vector<std::vector<double>>& left_eef_trajectory,
                std::vector<std::vector<int>>& left_eef_state_vector,
                std::vector<std::vector<double>>& object_positions_vector,
                std::ofstream& the_eef_file,
                std::ofstream& the_obj_file);

/*Recording method that uses all relevant parameters to control starting/stopping of recording
 * input: the Data_config class
 * return: nothing but it record the trajectory and object position in relevant files
 * */
void record_traj_and_object_position(Data_config& parameters,
                                     std::vector<std::vector<double> > &left_eef_trajectory,
                                     std::vector<std::vector<int> > &left_eef_state,
                                     ros::Publisher &image_pub,
                                     std::ofstream &the_file,
                                     std::ofstream &the_obj_file);

void reverse_left_eef_state(Data_config& parameters);
