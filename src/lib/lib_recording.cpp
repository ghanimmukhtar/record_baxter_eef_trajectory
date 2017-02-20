#include "../../include/record_baxter_eef_trajectory/lib_recording.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <cmath>        // std::abs

//receive the image and prepare it for manipulation
void config_pic(const sensor_msgs::ImageConstPtr& msg, Data_config& parameters){
    parameters.set_rgb_msg(msg);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        parameters.set_cv_pridged(cv_ptr);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    parameters.set_cv_image(parameters.get_cv_pridge()->image);
}

//manipulate image to recognize the marker and draw a circle around the middle of the marker
void config_and_detect_markers(Data_config& parameters){
    parameters.get_aruco_detector().setDictionary("ARUCO");
    parameters.get_aruco_detector().detect(parameters.get_cv_image(), parameters.get_markers(), parameters.get_camera_char(), parameters.get_marker_size());
    if (!parameters.get_markers().empty()){
        int index;
        std::vector<aruco::Marker>::iterator marker_itr;
        for(marker_itr = parameters.get_markers().begin(); marker_itr != parameters.get_markers().end(); marker_itr++){
            if((*marker_itr).id == 3)
                index = 0;
            else if((*marker_itr).id == 341)
                index = 1;
            (*marker_itr).draw(parameters.get_cv_image(), cv::Scalar(94.0, 206.0, 165.0, 0.0));
            (*marker_itr).calculateExtrinsics(parameters.get_marker_size(), parameters.get_camera_char(), false);
            Eigen::Vector2i marker_center;
            marker_center << (int) ((*marker_itr)[0].x + (*marker_itr)[2].x)/2,
                    (int) ((*marker_itr)[0].y + (*marker_itr)[2].y)/2;
            parameters.set_marker_center(marker_center, index);
            circle(parameters.get_cv_image(), cv::Point(((*marker_itr)[0].x + (*marker_itr)[2].x)/2,
                    ((*marker_itr)[0].y + (*marker_itr)[2].y)/2), 10, CV_RGB(255,0,0));

        }
    }
}

//show marker with circle around tracked point
void show_marker(Data_config& parameters){
    cv::namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);
    cv::imshow("ShowMarker", parameters.get_cv_image());
    cv::waitKey(1);
}

//locate object position
void locate_object(const sensor_msgs::ImageConstPtr& depth_msg, Data_config& parameters){
        if(!parameters.get_cv_image().empty() && !parameters.get_markers().empty()){
            rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, parameters.get_rgb_msg(), parameters.get_info_msg());
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
            image_processing::PointCloudT::Ptr input_cloud(new image_processing::PointCloudT);
            pcl::fromROSMsg(ptcl_msg, *input_cloud);
            ROS_ERROR_STREAM("the markers vector size is: " << parameters.get_markers().size());
            int i;
            for(int j = 0; j < parameters.get_number_of_markers(); j++){
                if(parameters.get_markers()[j].id == 3)
                    i = 0;
                else if(parameters.get_markers()[j].id == 341)
                    i = 1;
                image_processing::PointT pt = input_cloud->at((int) parameters.get_marker_center(i)(0) + (int) parameters.get_marker_center(i)(1)*input_cloud->width);
                Eigen::Vector3d object_position(pt.x, pt.y, pt.z);
                if(object_position[0] == object_position[0] && object_position[1] == object_position[1] && object_position[2] == object_position[2])
                    parameters.set_object_position(object_position, i);
            }
            for(size_t s = 0; s < parameters.get_object_position_vector().size(); s++)
                ROS_ERROR_STREAM("locate_object " << s << " : " << parameters.get_object_position_vector()[s]);
            ROS_ERROR_STREAM(" ");
        }
}

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

//Convert object position from camera frame to robot frame
void convert_object_position_to_robot_base(Eigen::Vector3d& object_pose_in_camera_frame, Eigen::Vector3d& object_pose_in_robot_frame){
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    //std::string child_frame = "/camera_depth_optical_frame";
    std::string child_frame = "/camera_rgb_optical_frame";
    std::string parent_frame = "/world";
    /*try{
        listener.lookupTransform(child_frame, parent_frame,
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }*/

    geometry_msgs::PointStamped camera_point;
    geometry_msgs::PointStamped base_point;
    camera_point.header.frame_id = child_frame;

    //we'll just use the most recent transform available for our simple example
    camera_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    camera_point.point.x = object_pose_in_camera_frame(0);
    camera_point.point.y = object_pose_in_camera_frame(1);
    camera_point.point.z = object_pose_in_camera_frame(2);

    ros::Duration my_duration(0.03);
    try{

        //listener.waitForTransform(parent_frame, child_frame, ros::Time::now(), my_duration);
        listener.transformPoint(parent_frame, camera_point, base_point);

        ROS_INFO("camera_depth_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                 camera_point.point.x, camera_point.point.y, camera_point.point.z,
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
    }
    object_pose_in_robot_frame << base_point.point.x,
            base_point.point.y,
            base_point.point.z;
}

//Display some images on baxter screen to reflect what he is doing
void dispaly_image(std::string path, ros::Publisher& image_pub){
    cv::Mat img = cv::imread(path);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    image_pub.publish(msg);
}

void convert_whole_object_positions_vector(std::vector<Eigen::Vector4d>& object_positions_vector, std::vector<std::vector<double>>& output_of_conversion){
    Eigen::Matrix4d Trans_M;


    Trans_M <<
            -0.100989,   0.714561,  -0.692246,  1.338,
            0.994465,  0.0927816, -0.0493054,  0.205,
            0.0289959,  -0.693393,  -0.719976,  0.301,
            0.0, 0.0, 0.0, 1.0;

    std::vector<Eigen::Vector4d>::iterator itr;
    for(itr = object_positions_vector.begin(); itr != object_positions_vector.end(); itr++){
        Eigen::Vector4d opv = Trans_M * (*itr);
        std::vector<double> inner_output;
        inner_output.push_back(opv(0));
        inner_output.push_back(opv(1));
        inner_output.push_back(opv(2));
        output_of_conversion.push_back(inner_output);
    }
}

void write_data(Data_config& parameters,
                std::vector<std::vector<double>>& left_eef_trajectory,
                std::vector<std::vector<double>>& object_positions_vector, std::ofstream& the_file){
    std::vector<std::vector<double>>::iterator outter_itr;
    std::vector<std::vector<double>>::iterator outter_itr_object = object_positions_vector.begin();
    for(outter_itr = left_eef_trajectory.begin();
        outter_itr != left_eef_trajectory.end();
        outter_itr++){
        std::vector<double>::iterator inner_itr;
        for(inner_itr = (*outter_itr).begin(); inner_itr != (*outter_itr).end(); inner_itr++) // eef pos and rotation
            the_file << (*inner_itr) << ",";

        for(int i = 0; i < parameters.get_number_of_markers(); i++){ // obj position per object
            for(inner_itr = (*outter_itr_object).begin(); inner_itr != (*outter_itr_object).end(); inner_itr++)
                the_file << (*inner_itr) << ",";
            the_file << 0 << "," << 0 << "," << 0 << ","; // obj rotation
            ++outter_itr_object;
        }
    }

    the_file << "\n";
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
    std::vector<Eigen::Vector4d> object_position_vector;
    std::vector<Eigen::Vector3d> prev_object_position_vector(parameters.get_number_of_markers());
    int nb_iter = 0;
    while(ros::ok()){
        int followed_object_index = 0;
        //ROS_INFO("go go go");
        //dispaly_image(not_working_image_path, image_pub);
        if(parameters.get_pressed() && parameters.get_lower_botton_pressed()){
            if(parameters.get_release()){
                parameters.set_release(false);
                parameters.set_toggle(true);
            }

            //wait till there is no (NaN values)
            //            if (parameters.get_number_of_markers() == 1) {
            while(parameters.get_object_position(followed_object_index)(0) != parameters.get_object_position(followed_object_index)(0) ||
                  parameters.get_object_position(followed_object_index)(1) != parameters.get_object_position(followed_object_index)(1) ||
                  parameters.get_object_position(followed_object_index)(2) != parameters.get_object_position(followed_object_index)(2))
                ROS_ERROR("I am in waiting limbo for 1 object...........");
            //            }
            //            else if (parameters.get_number_of_markers() == 2) {
            //                while(parameters.get_object_position(0)(0) != parameters.get_object_position(0)(0) ||
            //                      parameters.get_object_position(0)(1) != parameters.get_object_position(0)(1) ||
            //                      parameters.get_object_position(0)(2) != parameters.get_object_position(0)(2) ||
            //                      parameters.get_object_position(1)(0) != parameters.get_object_position(1)(0) ||
            //                      parameters.get_object_position(1)(1) != parameters.get_object_position(1)(1) ||
            //                      parameters.get_object_position(1)(2) != parameters.get_object_position(1)(2))
            //                    ROS_ERROR_STREAM("I am in waiting limbo for 2 objects...........");
            //            } else{
            //                ROS_ERROR("record_traj_and_object_position - wrong number of objects");
            //                exit(-1);
            //            }

            //get current eef pose
            Eigen::VectorXd current_values(6);
            current_values = parameters.get_left_eef_pose_rpy();
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

                //while saving end effector changes register object position concurrently
                for(int i = 0; i < parameters.get_number_of_markers(); i++){
                    Eigen::Vector3d current_obj_pos = parameters.get_object_position(i);
                    // If NaN get previous value
                    ROS_ERROR_STREAM("\n\n iteration" << nb_iter << " object ID " << i);
                    ROS_ERROR_STREAM(current_obj_pos[0] << " " << current_obj_pos[1] << " " << current_obj_pos[2]);
                    //                    ROS_ERROR_STREAM(prev_object_position_vector[i][0] << " " << prev_object_position_vector[i][1] << " " << prev_object_position_vector[i][2]);
                    //                    if (nb_iter > 0){
                    //                        for (int pos_coord = 0; pos_coord < 3; pos_coord++){
                    //                            ROS_ERROR_STREAM("resta " << std::abs(current_obj_pos[pos_coord] - prev_object_position_vector[i][pos_coord]));
                    //                            if (std::abs(current_obj_pos[pos_coord] - prev_object_position_vector[i][pos_coord]) > 0.01){
                    //                                current_obj_pos[pos_coord] = prev_object_position_vector[i][pos_coord];
                    //                                ROS_ERROR_STREAM("record_traj_and_object_position - obj pos coord " << i << " " << pos_coord << " problem");
                    //                            }
                    //                        }
                    //                    }

                    Eigen::Vector4d object_position_in_robot_frame;
                    object_position_in_robot_frame << current_obj_pos, 1;
                    object_position_vector.push_back(object_position_in_robot_frame);
                    prev_object_position_vector[i][0] = current_obj_pos[0];
                    prev_object_position_vector[i][1] = current_obj_pos[1];
                    prev_object_position_vector[i][2] = current_obj_pos[2];
                }
                ROS_ERROR_STREAM("this iteration duration is: " << ros::Time::now().toSec() - time_now);
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
            //left_eef_trajectory_file << "\n";
            //object_file << "\n";
            parameters.set_toggle(false);
            std::vector<std::vector<double>> output_of_conversion;
            convert_whole_object_positions_vector(object_position_vector, output_of_conversion);
            write_data(parameters, left_eef_trajectory, output_of_conversion, the_file);
            left_eef_trajectory.clear();
            object_position_vector.clear();
        }
        rate.sleep();
    }


}
