#include <record_baxter_eef_trajectory/lib_recording.hpp>

//get baxter left eef pose
void locate_left_eef_pose(baxter_core_msgs::EndpointState& l_eef_feedback, Data_config& parameters){
    Eigen::VectorXd left_end_effector_pose(6);
    parameters.set_left_eef_pose_quat(l_eef_feedback.pose);

    tf::quaternionMsgToTF(parameters.get_left_eef_pose_quat().orientation, parameters.get_left_eef_rpy_orientation());

    double roll, yaw, pitch;
    tf::Matrix3x3 m(parameters.get_left_eef_rpy_orientation());
    m.getRPY(roll, pitch, yaw);
    left_end_effector_pose << l_eef_feedback.pose.position.x,
            l_eef_feedback.pose.position.y,
            l_eef_feedback.pose.position.z,
            roll,
            pitch,
            yaw;

    parameters.set_left_eef_pose_rpy(left_end_effector_pose);

}

void get_angles_from_rotation_matrix(tf::Matrix3x3& rotation_matrix,
                                     std::vector<double>& returned_angles,
                                     std::string my_type){
    double roll, pitch, yaw;
    tf::Quaternion my_angles;
    returned_angles.clear();
    if(strcmp(my_type.c_str(), "RPY") == 0){
        rotation_matrix.getRPY(roll, pitch, yaw);
        returned_angles.push_back(roll);
        returned_angles.push_back(pitch);
        returned_angles.push_back(yaw);
    }
    else{
        rotation_matrix.getRotation(my_angles);
        returned_angles.push_back(my_angles.getX());
        returned_angles.push_back(my_angles.getY());
        returned_angles.push_back(my_angles.getZ());
        returned_angles.push_back(my_angles.getW());
    }
}


void write_data(Data_config& parameters,
                std::vector<std::vector<double>>& left_eef_trajectory,
                std::vector<std::vector<double>>& object_positions_vector, std::ofstream& the_file){
    ROS_INFO_STREAM("TRAJECTORY RECORDER: Trying to write data, eef traje is size: "
                    << left_eef_trajectory.size()
                    << " and object_position is: "
                    << object_positions_vector.size());

    std::vector<std::vector<double>>::iterator outter_itr;
    std::vector<std::vector<double>>::iterator outter_itr_object = object_positions_vector.begin();
    for(outter_itr = left_eef_trajectory.begin();
        outter_itr != left_eef_trajectory.end();
        outter_itr++){
        std::vector<double>::iterator inner_itr;
        for(inner_itr = (*outter_itr).begin(); inner_itr != (*outter_itr).end(); inner_itr++) // eef pos and rotation
            the_file << (*inner_itr) << ",";

        for(size_t i = 0; i < parameters.get_objects_positions_map().size(); i++){ // obj position per object
            for(inner_itr = (*outter_itr_object).begin(); inner_itr != (*outter_itr_object).end(); inner_itr++)
                the_file << (*inner_itr) << ",";
            the_file << 0 << "," << 0 << "," << 0 << ","; // obj rotation
            ++outter_itr_object;
        }
    }

    the_file << "\n";
}

void record_object_position(const visual_functionalities::object_qr_positionConstPtr& object_position_topic,
                            Data_config& parameters){
    //ROS_INFO("TRAJECTORY RECORDER: Hello I am recording object position");
    //if the map is empty, this is first time to insert then just insert
    if(parameters.get_objects_positions_map().find(object_position_topic->qr_id.data) == parameters.get_objects_positions_map().end() &&
            parameters.get_objects_positions_map().size() != parameters.get_number_of_markers()){
        parameters.set_markers_id_vector(object_position_topic->qr_id.data, parameters.get_objects_positions_map().size());
        std::vector<std::vector<double>> object_positions;
        object_positions.push_back({object_position_topic->object_qr_position.point.x,
                                    object_position_topic->object_qr_position.point.y,
                                    object_position_topic->object_qr_position.point.z});
        parameters.get_objects_positions_map().insert( std::pair<int, std::vector<std::vector<double> > >(object_position_topic->qr_id.data, object_positions) );
    }
    else if(parameters.get_objects_positions_map().find(object_position_topic->qr_id.data) !=
            parameters.get_objects_positions_map().end()){
//        ROS_INFO_STREAM("TRAJECTORY_RECORDER: Recording position for id: " << object_position_topic->qr_id.data);
//        ROS_INFO_STREAM("TRAJECTORY_RECORDER: X is: " << object_position_topic->object_qr_position.point.x);
//        ROS_INFO_STREAM("TRAJECTORY_RECORDER: Y is: " << object_position_topic->object_qr_position.point.y);
//        ROS_INFO_STREAM("TRAJECTORY_RECORDER: Z is: " << object_position_topic->object_qr_position.point.z);
//        ROS_WARN("********************************************");
        parameters.get_objects_positions_map().find(object_position_topic->qr_id.data)->second.push_back(
        {object_position_topic->object_qr_position.point.x,
         object_position_topic->object_qr_position.point.y,
         object_position_topic->object_qr_position.point.z});
    }
    else
        ROS_WARN("TRAJECTORY_RECORDER: This qr code is not among the objects to be tracked");
}

void convert_whole_object_positions_vector(Data_config& parameters,
                                           std::vector<std::vector<double>>& object_positions_vector,
                                           std::vector<std::vector<double>>& output_of_conversion){
    if(object_positions_vector.empty()){
        ROS_ERROR("THE TRANSFORMATION IS IMPOSSIBLE, EMPTY VECTOR");
        return;
    }
    ROS_INFO_STREAM("TRAJECTORY_RECORDER: Trying to convert, the size is: " << object_positions_vector.size());
    output_of_conversion.resize(object_positions_vector.size());
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    std::string child_frame = parameters.get_child_frame();
    std::string parent_frame = parameters.get_parent_frame();
    try{
        listener.lookupTransform(child_frame, parent_frame,
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::vector<geometry_msgs::PointStamped> camera_point;
    std::vector<geometry_msgs::PointStamped> base_point;
    camera_point.resize(object_positions_vector.size());
    base_point.resize(object_positions_vector.size());

    for(size_t i = 0; i < camera_point.size(); i++){
        camera_point[i].header.frame_id = child_frame;

        //we'll just use the most recent transform available for our simple example
        camera_point[i].header.stamp = ros::Time();

        camera_point[i].point.x = object_positions_vector[i][0];
        camera_point[i].point.y = object_positions_vector[i][1];
        camera_point[i].point.z = object_positions_vector[i][2];

        try{
            listener.transformPoint(parent_frame, camera_point[i], base_point[i]);
//            ROS_INFO("kinect2_rgb_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
//                     camera_point[i].point.x, camera_point[i].point.y, camera_point[i].point.z,
//                     base_point[i].point.x, base_point[i].point.y, base_point[i].point.z, base_point[i].header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
        }
        output_of_conversion[i] = {base_point[i].point.x, base_point[i].point.y, base_point[i].point.z};
    }
}

//record marker position (object position) if changed in the specified file
void record_traj_and_object_position(Data_config& parameters,
                                     std::vector<std::vector<double>>& left_eef_trajectory,
                                     ros::Publisher& image_pub,
                                     std::ofstream& the_file){

    Eigen::VectorXd old_values(6);
    old_values = parameters.get_left_eef_pose_rpy();
    //ROS_ERROR_STREAM("here old values are: " << old_values);
    parameters.set_pressed(false);
    parameters.set_release(true);
    parameters.set_toggle(false);
    std::string working_image_path = "/home/ghanim/git/catkin_ws/src/baxter_examples/share/images/working.jpg";
    std::string not_working_image_path = "/home/ghanim/git/catkin_ws/src/baxter_examples/share/images/finished.jpg";
    ros::Rate rate(parameters.get_the_rate());
    rate.sleep();
    double time_now = 0.0;
    std::vector<std::vector<double> > object_position_vector;
    std::vector<Eigen::Vector3d> prev_object_position_vector(parameters.get_number_of_markers());
    int nb_iter = 0;
    while(ros::ok()){
        //ROS_INFO("go go go");
        //dispaly_image(not_working_image_path, image_pub);
        if(parameters.get_pressed() && parameters.get_lower_botton_pressed()){
//        if(parameters.get_start_recording() == 1){
            if(parameters.get_release()){
                parameters.set_release(false);
                parameters.set_toggle(true);
            }

            //get current eef pose
            Eigen::VectorXd current_values(6);
            current_values = parameters.get_left_eef_pose_rpy();
            //ROS_INFO_STREAM("TRAJECTORY RECORDER: EEF pose is: " << current_values);
            if ((current_values - old_values).norm() > parameters.get_epsilon()){
                time_now = ros::Time::now().toSec();

                std::vector<double> inner_left_traj;
                inner_left_traj.push_back(current_values(0));
                inner_left_traj.push_back(current_values(1));
                inner_left_traj.push_back(current_values(2));
                inner_left_traj.push_back(current_values(3));
                inner_left_traj.push_back(current_values(4));
                inner_left_traj.push_back(current_values(5));
                left_eef_trajectory.push_back(inner_left_traj);
                old_values = current_values;

                if(parameters.get_objects_positions_map().empty())
                    for(int i = 0; i < parameters.get_number_of_markers(); i++)
                        object_position_vector.push_back({0, 0, 0});
                else if(parameters.get_objects_positions_map().size() != parameters.get_number_of_markers()){
//                    ROS_WARN_STREAM("TRAJECTORY_RECORDER: There are: " << parameters.get_objects_positions_map().size()
//                                    << " element in the map, the size of vector of vector is: " );
//                                    for(auto& x: parameters.get_objects_positions_map())
//                                    ROS_WARN_STREAM("TRAJECTORY_RECORDER: " << x.second.size());
                    for(auto& x: parameters.get_objects_positions_map()){
                            //ROS_WARN_STREAM("TRAJECTORY_RECORDER: Size of object position vector is: " << x.second[x.second.size() - 1].size());
                        object_position_vector.push_back(x.second[x.second.size() - 1]);
                    }
                    for(int i = parameters.get_objects_positions_map().size(); i < parameters.get_number_of_markers(); i++)
                        object_position_vector.push_back({0, 0, 0});
                }
                else if(parameters.get_objects_positions_map().size() == parameters.get_number_of_markers()){
                    for(auto& x: parameters.get_objects_positions_map()){
                        ROS_WARN_STREAM("TRAJECTORY_RECORDER: Storing the object position with id: "
                                        << x.first);
                        for(size_t i = 0; i < x.second[x.second.size() - 1].size(); i++)
                            ROS_INFO_STREAM("TRAJECTORY_RECORDER: Element" << i << " is: "
                                            << x.second[x.second.size() - 1][i]);

                        ROS_WARN("********************************************");
                        object_position_vector.push_back(x.second[x.second.size() - 1]);
                    }
                }

                //ROS_ERROR_STREAM("this iteration duration is: " << ros::Time::now().toSec() - time_now);
            }
            nb_iter++;
        }
        else{
            if(parameters.get_toggle()){
                ROS_ERROR("I need two buttons to be pressed simultaneously ...........");
                parameters.set_release(true);
                parameters.set_lower_button_pressed(false);

            }
        }
        if(parameters.get_toggle() && !parameters.get_lower_botton_pressed()){
//        if(parameters.get_start_recording() == 2){
            parameters.set_start_recording(0);
            parameters.set_toggle(false);
            std::vector<std::vector<double>> output_of_conversion;
            ROS_INFO_STREAM("TRAJECTORY RECORDER: EEF pose vector size is: " << left_eef_trajectory.size());
            ROS_INFO_STREAM("TRAJECTORY RECORDER: Object position map size is: " << parameters.get_objects_positions_map().size());
            if(!parameters.get_objects_positions_map().empty())
                convert_whole_object_positions_vector(parameters, object_position_vector, output_of_conversion);

            write_data(parameters, left_eef_trajectory, output_of_conversion, the_file);
            left_eef_trajectory.clear();
            object_position_vector.clear();
            parameters.get_objects_positions_map().clear();
        }
        rate.sleep();
    }


}
