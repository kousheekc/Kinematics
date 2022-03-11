#include "Kinematics.h"
#include "MatrixUtils.h"

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    Kinematics kin(3);
    MatrixUtils mat_utils;

    kin.add_joint_axis(0, 0,  1,  4, 0,    0);
    kin.add_joint_axis(0, 0,  0,  0, 1,    0);
    kin.add_joint_axis(0, 0, -1, -6, 0, -0.1);

    kin.add_initial_end_effector_pose(-1, 0,  0, 0,
                                       0, 1,  0, 6,
                                       0, 0, -1, 2,
                                       0, 0,  0, 1);

    float joint_angles[3] = {PI/2.0, 3, PI};
    float transform[4][4];

    kin.forward(joint_angles, (float*)transform);
    mat_utils.print_matrix((float*)transform, 4, 4, "Transform");
}

void loop() {
    // put your main code here, to run repeatedly:

}
