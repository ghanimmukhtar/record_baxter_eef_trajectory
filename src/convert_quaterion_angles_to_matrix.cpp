#include "record_baxter_eef_trajectory/lib_recording.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_quaterion_angles_to_matrix");
    ros::NodeHandle n;

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

    Data_config params;
    n.getParam("camera_pose", params.get_camera_frame_pose());
    n.getParam("camera_frame_choice", params.get_camera_frame_choice());

    set_camera_poses_and_transformation(params);
    return 0;
}
