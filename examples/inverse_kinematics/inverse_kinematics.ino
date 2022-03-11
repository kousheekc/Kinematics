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

    float desired_transform[4][4] = {
        {0, 1,  0,     -5},
        {1, 0,  0,      4},
        {0, 0, -1, 1.6858},
        {0, 0,  0,      1}
    };

    float jac[6][N];
    float jac_t[6][N];
    float AA_t[6][6];
    float A_tA[N][N];
    float pinv[N][6];

    float joint_angles_0[N] = {1.0, 2.5, 3};
    float joint_angles[N];

    kin.inverse((float*)desired_transform, (float*)jac, (float*)pinv, (float*)jac_t, (float*)AA_t, (float*)A_tA, joint_angles_0, 0.01, 0.001, 20, joint_angles);
    mat_utils.print_matrix(joint_angles, 1, N, "Joint angles");
}

void loop() {
  // put your main code here, to run repeatedly:

}
