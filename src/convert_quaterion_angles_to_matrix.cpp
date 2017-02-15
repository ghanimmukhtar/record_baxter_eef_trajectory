#include "record_baxter_eef_trajectory/lib_recording.hpp"

int main(){
    tf::Quaternion my_angles;
//    my_angles.setW(-0.016);
//    my_angles.setX(-0.438);
//    my_angles.setY(-0.031);
//    my_angles.setZ(0.898);
    my_angles.setRPY(-2.297, 0.026, 1.561);

    tf::Matrix3x3 rotation_matrix(my_angles);
    Eigen::Matrix3d output;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++){
            ROS_INFO_STREAM("element in row: " << i << " column: " << j << " is: " << rotation_matrix[i][j]);

        }
    output << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2];
    ROS_INFO_STREAM("the whole matrix is: \n" << output);
    return 0;
}
