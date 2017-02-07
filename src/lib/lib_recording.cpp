#include "../../include/record_baxter_eef_trajectory/lib_recording.hpp"

#include <opencv2/highgui/highgui.hpp>

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
        //ROS_WARN_STREAM("marker size is: " << markers[0].size());
        parameters.get_markers()[0].draw(parameters.get_cv_image(), cv::Scalar(94.0, 206.0, 165.0, 0.0));
        parameters.get_markers()[0].calculateExtrinsics(parameters.get_marker_size(), parameters.get_camera_char(), false);
        Eigen::Vector2i marker_center;
        marker_center << (int) (parameters.get_markers()[0][0].x + parameters.get_markers()[0][2].x)/2,
                (int) (parameters.get_markers()[0][0].y + parameters.get_markers()[0][2].y)/2;
        parameters.set_marker_center(marker_center);
        circle(parameters.get_cv_image(), cv::Point((parameters.get_markers()[0][0].x + parameters.get_markers()[0][2].x)/2,
                (parameters.get_markers()[0][0].y + parameters.get_markers()[0][2].y)/2), 10, CV_RGB(255,0,0));
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
        image_processing::PointT pt = input_cloud->at((int) parameters.get_marker_center()(0) + (int) parameters.get_marker_center()(1)*input_cloud->width);
        Eigen::Vector3d object_position(pt.x, pt.y, pt.z);
        parameters.set_object_position(object_position);
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
    left_end_effector_pose << parameters.get_left_eef_pose_quat().position.x,
            parameters.get_left_eef_pose_quat().position.y,
            parameters.get_left_eef_pose_quat().position.z,
            roll,
            pitch,
            yaw;

    parameters.set_left_eef_pose_rpy(left_end_effector_pose);

}

void convert_object_position_to_robot_base(Eigen::Vector3d& object_pose_in_camera_frame, Eigen::Vector3d& object_pose_in_robot_frame){
    tf::TransformListener listener;
    geometry_msgs::PointStamped camera_point;
    geometry_msgs::PointStamped base_point;
    camera_point.header.frame_id = "camera_link";

    //we'll just use the most recent transform available for our simple example
    camera_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    camera_point.point.x = object_pose_in_camera_frame(0);
    camera_point.point.y = object_pose_in_camera_frame(1);
    camera_point.point.z = object_pose_in_camera_frame(2);

    try{

        listener.transformPoint("base", camera_point, base_point);

        ROS_INFO("camera_link: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                 camera_point.point.x, camera_point.point.y, camera_point.point.z,
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }
    object_pose_in_robot_frame << base_point.point.x,
            base_point.point.y,
            base_point.point.z;
}

//record marker position (object position) if changed in the specified file
void record_traj_and_object_position(Data_config& parameters, std::ofstream& left_eef_trajectory_file, std::ofstream& object_file){
    /*std::string pPath;
    pPath = getenv ("PWD");
    std::string path_to_file;
    path_to_file.append(pPath);
    path_to_file.append("eef_trajectory_recorder");*/

    Eigen::VectorXd old_values(6);
    old_values = parameters.get_left_eef_pose_rpy();
    //ROS_ERROR_STREAM("here old values are: " << old_values);
    parameters.set_pressed(false);
    parameters.set_release(true);
    parameters.set_toggle(false);



    if(object_file.is_open() && left_eef_trajectory_file.is_open()){
        ros::Rate rate(50.0);
        rate.sleep();
        while(ros::ok()){
        //ROS_INFO("go go go");

            if(parameters.get_pressed() && parameters.get_lower_botton_pressed()){
                if(parameters.get_release()){
                    parameters.set_release(false);
                    parameters.set_toggle(true);
                }
                //wait till there is no (NaN values)
                while(parameters.get_object_position()(0) != parameters.get_object_position()(0) ||
                      parameters.get_object_position()(1) != parameters.get_object_position()(1) ||
                      parameters.get_object_position()(2) != parameters.get_object_position()(2)){
                    ROS_ERROR("I am in waiting limbo ...........");
                }

                //get current eef pose
                Eigen::VectorXd current_values(6);
                current_values = parameters.get_left_eef_pose_rpy();
                if ((current_values - old_values).norm() > parameters.get_epsilon()){
                    ROS_ERROR("I am recording ...........");
                    left_eef_trajectory_file << current_values(0) << ","
                                             << current_values(1) << ","
                                             << current_values(2) << ","
                                             << current_values(3) << ","
                                             << current_values(4) << ","
                                             << current_values(5) << ",";
                    old_values = current_values;
                    //while saving end effector changes register object position concurrently
                    Eigen::Vector3d object_position_in_robot_frame;
                    convert_object_position_to_robot_base(parameters.get_object_position(), object_position_in_robot_frame);
                    object_file << object_position_in_robot_frame(0) << ","
                                << object_position_in_robot_frame(1) << ","
                                << object_position_in_robot_frame(2) << ",";
                }

            }
            else{
                if(parameters.get_toggle()){
                    ROS_ERROR("I need two buttons to be pressed simultaneously ...........");
                    parameters.set_release(true);
                    parameters.set_lower_button_pressed(false);
                }
            }
            if(parameters.get_toggle() && !parameters.get_pressed()){
                left_eef_trajectory_file << "\n";
                object_file << "\n";
                parameters.set_toggle(false);
            }
            rate.sleep();
        }
    }

}
