#include "record_baxter_eef_trajectory/lib_recording.hpp"

int main(){
    tf::Quaternion my_angles;
//    my_angles.setW(-0.016);
//    my_angles.setX(-0.438);
//    my_angles.setY(-0.031);
//    my_angles.setZ(0.898);
    my_angles.setRPY(-0.066, 0.907, -3.137);

    tf::Matrix3x3 rotation_matrix(my_angles);

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            ROS_INFO_STREAM("element in row: " << i << " column: " << j << " is: " << rotation_matrix[i][j]);

    return 0;
}
