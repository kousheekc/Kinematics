#include "Kinematics.h"
#include "MatrixUtils.h"

#define N 3

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    Kinematics kin(N);
    MatrixUtils mat_utils;

    kin.add_joint_axis(0, 0,  1,  4, 0,    0);
    kin.add_joint_axis(0, 0,  0,  0, 1,    0);
    kin.add_joint_axis(0, 0, -1, -6, 0, -0.1);

    kin.add_initial_end_effector_pose(-1, 0,  0, 0,
                                       0, 1,  0, 6,
                                       0, 0, -1, 2,
                                       0, 0,  0, 1);

    float joint_angles[N] = {PI/2.0, 3, PI};
    float transform[4][4];

    kin.forward(joint_angles, (float*)transform);
    mat_utils.print_matrix((float*)transform, 4, 4, "Transform");

    // Output
    // Transform
    // 0.00    1.00    0.00    -5.00
    // 1.00    -0.00   0.00    4.00
    // 0.00    0.00    -1.00   1.69
    // 0.00    0.00    0.00    1.00
}

void loop() {
    // put your main code here, to run repeatedly:

}
