#include "record_baxter_eef_trajectory/lib_recording.hpp"

// The parameters structure is used by all call backs, main and service
Data_config parameters;

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

void object_qr_position_Callback(const visual_functionalities::object_qr_positionConstPtr& object_position){
    if(parameters.get_pressed() && parameters.get_lower_botton_pressed())
    //if(parameters.get_start_recording() == 1)
        record_qr_position(object_position, parameters);
}

void object_blob_position_Callback(const visual_functionalities::object_blob_positionConstPtr& object_position){
    if(parameters.get_pressed() && parameters.get_lower_botton_pressed()){
        ROS_INFO("INDICATIVE TEXT: I am recording blobs");
    //if(parameters.get_start_recording() == 1)
        record_blob_position(object_position, parameters);
    }
}

void start_recording_Callback(const std_msgs::Int64ConstPtr& start){
    parameters.set_start_recording(start->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_left_eef_trajectory");
  ros::NodeHandle n;

  //subscribers
  ros::Subscriber sub_l_eef_msg = n.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
  ros::Subscriber sub_l_cuf_msg = n.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 10, left_cuf_Callback);
  ros::Subscriber sub_l_lower_button = n.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_button/state", 10, left_lower_button_Callback);
  ros::Subscriber object_qr_position_sub = n.subscribe<visual_functionalities::object_qr_position>("/object_qr_position", 10, object_qr_position_Callback);
  ros::Subscriber object_blob_position_sub = n.subscribe<visual_functionalities::object_blob_position>("/object_blob_position", 10, object_blob_position_Callback);
  //ros::Subscriber start_recording_sub = n.subscribe<std_msgs::Int64>("/start_recording", 10, start_recording_Callback);
  ros::Publisher image_publisher = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);

  ros::AsyncSpinner my_spinner(4);
  my_spinner.start();
  usleep(1e6);

  std::string feedback_eef_file, feedback_obj_file;
  double the_rate;
  n.getParam("the_rate", the_rate);
  n.getParam("detection_method", parameters.get_detection_method());
  n.getParam("epsilon", parameters.get_epsilon());
  n.getParam("feedback_eef_file", feedback_eef_file);
  n.getParam("feedback_obj_file", feedback_obj_file);
  n.getParam("child_frame", parameters.get_child_frame());
  n.getParam("parent_frame", parameters.get_parent_frame());
  parameters.set_the_rate(the_rate);

  std::ofstream left_eef_trajectory_file(feedback_eef_file, std::ofstream::out | std::ofstream::trunc);
  std::ofstream obj_trajectory_file(feedback_obj_file, std::ofstream::out | std::ofstream::trunc);

  std::vector<std::vector<double>> left_eef_trajectory;

  record_traj_and_object_position(parameters, left_eef_trajectory, image_publisher, left_eef_trajectory_file, obj_trajectory_file);

  left_eef_trajectory_file.close();
  obj_trajectory_file.close();

  return 0;
}
