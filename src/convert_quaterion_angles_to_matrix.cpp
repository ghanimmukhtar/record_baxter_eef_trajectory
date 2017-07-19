#include "record_baxter_eef_trajectory/lib_recording.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_quaterion_angles_to_matrix");
    ros::NodeHandle n;

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

    /*tf::Quaternion quat(-0.392, -0.0197, 0.949, 0.0723);
    tf::Matrix3x3 rotation(quat);
    double roll, bitch, yaw;
    rotation.getRPY(roll, bitch, yaw);
    ROS_INFO_STREAM("roll angle is: " << roll << ", and bitch is: " << bitch << " and yaw is: " << yaw);*/
    /*Data_config params;
    n.getParam("camera_pose", params.get_camera_frame_pose());
    n.getParam("camera_frame_choice", params.get_camera_frame_choice());

    set_camera_poses_and_transformation(params);
    */


    tf::Quaternion my_angles;
    my_angles.setRPY(0.0, 0.5, -3.14);
    my_angles.setW(0.500);
    my_angles.setX(0.500);
    my_angles.setY(-0.500);
    my_angles.setZ(0.500);
    tf::Matrix3x3 rotation_matrix(my_angles);

    Eigen::Matrix4d T_o_c;
    T_o_c << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],-0.02,
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], 0.0,
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], 0.0,
            0,                     0,                     0,              1;

    /*tf::Quaternion quat(-0.392, -0.0197, 0.949, 0.0723);
    tf::Matrix3x3 rotation(quat);*/

    Eigen::Matrix4d T_b_o;
    T_b_o <<    -1.0227, -0.000855236,   -0.0332196,     0.292469,
                0.0028847,     0.795614,    -0.671273,      0.66033,
                0.0547889,    -0.575098,    -0.685392,     0.740042,
                        0,            0,            0,            1;
    /*T_b_o <<  -0.006,  0.48, -0.877,  0.95,
              1.02, -0.05, -0.02, -0.03,
            0.03, -0.92, -0.6,  0.845,
                    0 ,        0,         0,         1;*/

    Eigen::Matrix4d T_b_c;
    /*T_b_c << rotation[0][0], rotation[0][1], rotation[0][2],0.85,
            rotation[1][0], rotation[1][1], rotation[1][2], -0.225,
            rotation[2][0], rotation[2][1], rotation[2][2], 0.813,
            0,                     0,                     0,              1;*/
    T_b_c = T_b_o * (T_o_c);
    ROS_ERROR_STREAM("transformation matrix between base and camera_link is: " << T_b_c);
    std::vector<double> my_angles_2;
    tf::Matrix3x3 rotation2(T_b_c(0,0),  T_b_c(0,1), T_b_c(0,2),
                            T_b_c(1,0), T_b_c(1,1), T_b_c(1,2),
                            T_b_c(2,0), T_b_c(2,1), T_b_c(2,2));

    /*tf::Matrix3x3 rotation(0.0550968,  0.688488, -0.792489,
                           1.09941, 0.0265071, 0.0708362,
                         0.0733808, -0.776962, -0.671882);*/

    /*ROS_ERROR_STREAM("Quaternion angles are: ");
    ROS_ERROR_STREAM("for x: " << my_angles.getX());
    ROS_ERROR_STREAM("for y: " << my_angles.getY());
    ROS_ERROR_STREAM("for z: " << my_angles.getZ());
    ROS_ERROR_STREAM("for w: " << my_angles.getW());*/
    get_angles_from_rotation_matrix(rotation2, my_angles_2);
    ROS_ERROR_STREAM("Quaternion angles are: ");
    ROS_ERROR_STREAM("for x: " << my_angles_2[0]);
    ROS_ERROR_STREAM("for y: " << my_angles_2[1]);
    ROS_ERROR_STREAM("for z: " << my_angles_2[2]);
    ROS_ERROR_STREAM("for w: " << my_angles_2[3]);
    /*ROS_ERROR_STREAM("RPY angles are:");
    ROS_ERROR_STREAM("for roll: " << my_angles_2[0]);
    ROS_ERROR_STREAM("for pitch: " << my_angles_2[1]);
    ROS_ERROR_STREAM("for yaw: " << my_angles_2[2]);*/


    return 0;
}
