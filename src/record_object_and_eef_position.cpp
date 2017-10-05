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
void left_lower_button_Callback(const baxter_core_msgs::DigitalIOStateConstPtr& l_lower_button_feedbcak){
    if(l_lower_button_feedbcak->state)
        parameters.set_lower_button_pressed(true);
}

//Reverse the status of left gripper when the upper button is pushed
void left_upper_button_Callback(const baxter_core_msgs::DigitalIOStateConstPtr& l_upper_button_feedbcak){
        if(l_upper_button_feedbcak->state){
            reverse_left_eef_state(parameters);
            usleep(1e6);
        }
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

void left_gripper_status_Callback(const baxter_core_msgs::EndEffectorStateConstPtr& eef_state){
    if(eef_state->position > 50){
//            ROS_INFO("LEFT GRIPPER IS OPEN");
            parameters.set_left_eef_state(1);
        }
    else{
//            ROS_INFO("LEFT GRIPPER IS CLOSED");
            parameters.set_left_eef_state(0);
        }
}

void right_gripper_status_Callback(const baxter_core_msgs::EndEffectorStateConstPtr& eef_state){
    if(eef_state->position > 50){
//            ROS_INFO("RIGHT GRIPPER IS OPEN");
            parameters.set_right_eef_state(1);
        }
    else{
//            ROS_INFO("RIGHT GRIPPER IS CLOSED");
            parameters.set_right_eef_state(0);
        }
}

//store obj pos vector in parameters
void obj_state_cloud_Callback(const pcl_tracking::ObjectPosition::ConstPtr& topic_message){
    if(parameters.get_pressed() && parameters.get_lower_botton_pressed()){
        std::vector< geometry_msgs::PointStamped > raw_pos_vector = topic_message->object_position;
        std::vector< std::vector<double> > obj_pos_vector(raw_pos_vector.size()-1, std::vector<double>(3));
        std::vector<double> curr_obj_pos;
        geometry_msgs::PointStamped curr_raw_obj;
        curr_raw_obj = raw_pos_vector[0];
        curr_obj_pos.push_back(curr_raw_obj.point.x);
        curr_obj_pos.push_back(curr_raw_obj.point.y);
        curr_obj_pos.push_back(curr_raw_obj.point.z);
        parameters.push_cloud_state_vector(curr_obj_pos);
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
  ros::Subscriber sub_l_upper_button = n.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_upper_button/state", 10, left_upper_button_Callback);
  ros::Subscriber object_qr_position_sub = n.subscribe<visual_functionalities::object_qr_position>("/object_qr_position", 10, object_qr_position_Callback);
  ros::Subscriber object_blob_position_sub = n.subscribe<visual_functionalities::object_blob_position>("/object_blob_position", 10, object_blob_position_Callback);
  ros::Subscriber left_gripper_status = n.subscribe<baxter_core_msgs::EndEffectorState>("/robot/end_effector/left_gripper/state", 10, left_gripper_status_Callback);
  ros::Subscriber right_gripper_status = n.subscribe<baxter_core_msgs::EndEffectorState>("/robot/end_effector/right_gripper/state", 10, right_gripper_status_Callback);
  ros::Publisher image_publisher = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);

  ros::Subscriber object_cloud_position_sub = n.subscribe<pcl_tracking::ObjectPosition>("/visual/obj_pos_vector", 1, obj_state_cloud_Callback);

  ros::ServiceClient left_gripper_ac = n.serviceClient<baxter_kinematics::GripperAction>("/baxter_kinematics/gripper_action");

  ros::AsyncSpinner my_spinner(4);
  my_spinner.start();
  usleep(1e6);

  parameters.set_left_eef_service_client(left_gripper_ac);
  std::string feedback_eef_file, feedback_obj_file, feedback_openness_file;
  double the_rate;
  n.getParam("the_rate", the_rate);
  n.getParam("detection_method", parameters.get_detection_method());
  n.getParam("epsilon", parameters.get_epsilon());
  n.getParam("feedback_eef_file", feedback_eef_file);
  n.getParam("feedback_obj_file", feedback_obj_file);
  n.getParam("feedback_openness_file", feedback_openness_file);
  n.getParam("child_frame", parameters.get_child_frame());
  n.getParam("parent_frame", parameters.get_parent_frame());
  parameters.set_the_rate(the_rate);

  bool append_record_file;
  n.getParam("append_record_file", append_record_file);
  std::ofstream left_eef_trajectory_file;
  std::ofstream obj_trajectory_file;
  std::ofstream openness_trajectory_file;
  std::vector<std::vector<double>> left_eef_trajectory;
  std::vector<std::vector<int>> left_eef_state;

  if (!append_record_file){
      left_eef_trajectory_file.open(feedback_eef_file, std::ofstream::out | std::ofstream::trunc);
      obj_trajectory_file.open(feedback_obj_file, std::ofstream::out | std::ofstream::trunc);
      openness_trajectory_file.open(feedback_openness_file, std::ofstream::out | std::ofstream::trunc);
  }
  else{
      left_eef_trajectory_file.open(feedback_eef_file, std::ofstream::out | std::ofstream::app);
      obj_trajectory_file.open(feedback_obj_file, std::ofstream::out | std::ofstream::app);
      openness_trajectory_file.open(feedback_openness_file, std::ofstream::out | std::ofstream::app);
  }

  record_traj_and_object_position(parameters,
                                  left_eef_trajectory,
                                  left_eef_state,
                                  image_publisher,
                                  left_eef_trajectory_file,
                                  obj_trajectory_file,
                                  openness_trajectory_file);

  left_eef_trajectory_file.close();
  obj_trajectory_file.close();
  openness_trajectory_file.close();

  return 0;
}
