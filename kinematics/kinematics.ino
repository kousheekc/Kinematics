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
                                    0, 1, 0, 6,
                                    0, 0, -1, 2,
                                    0, 0, 0, 1);

    float T[4][4] = {
        {0, 1,  0,     -5},
        {1, 0,  0,      4},
        {0, 0, -1, 1.6858},
        {0, 0,  0,      1}
    };

    float thetalist0[3] = {1.5, 2.5, 3};
    float thetalist[3];

    kin.inverse((float*)T, thetalist0, 0.01, 0.001, 20, thetalist);
    mat_utils.print_matrix(thetalist, 1, 3, "Joint angles");



    // kin.add_joint_axis(0, 0, 1, 0, 0, 0);
    // kin.add_joint_axis(0, 1, 0, -290, 0, 0);
    // kin.add_joint_axis(0, 1, 0, -560, 0, 0);
    // kin.add_joint_axis(1, 0, 0, 0, 630, 0);
    // kin.add_joint_axis(0, 1, 0, -302, 0, 630);
    // kin.add_joint_axis(1, 0, 0, 0, 630, 0);

    // kin.add_initial_end_effector_pose(0,  0,  1, 374,
    //                                   0, -1,  0, 0,
    //                                   1,  0,  0, 630,
    //                                   0,  0,  0, 1);

    // float joint_angles[6] = {0, 0, 0, 0, 0, 0};
    // float transform[4][4];
    // kin.forward(joint_angles, (float*)transform);
    // mat_utils.print_matrix((float*)transform, 4, 4, "T");
}

void loop() {
  // put your main code here, to run repeatedly:

}
