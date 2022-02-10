#include "Kinematics.h"
#include "Utilities.h"

void setup() {
// put your setup code here, to run once:
    Serial.begin(9600);

    // Kinematics kin(4);
    Utilities utils;

    float** m = utils.create_mat(4);
    m[0][0] = 1;
    m[0][1] = 0;
    m[0][2] = 0;
    m[0][3] = 0;

    m[1][0] = 0;
    m[1][1] = 0;
    m[1][2] = -1;
    m[1][3] = 0;

    m[2][0] = 0;
    m[2][1] = 1;
    m[2][2] = 0;
    m[2][3] = 3;

    m[3][0] = 0;
    m[3][1] = 0;
    m[3][2] = 0;
    m[3][3] = 1;

    float** result = utils.inverse(m);
    utils.print_mat(result, 4);

    // kin.add_joint_axis(0, 0, 1, 0, 0.2, 0.2);
    // kin.add_joint_axis(1, 0, 0, 2, 0, 3);
    // kin.add_joint_axis(0, 1, 0, 0, 2, 1);
    // kin.add_joint_axis(1, 0, 0, 0.2, 0.3, 0.4);

    // float joint_angles[4] = {0.2, 1.1, 0.1, 1.2};

    // float** jacobian = kin.jacobian(joint_angles);

    // utils.print_mat(jacobian, 6, 4);
    
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
    // float joint_angles[6] = {PI/2.0, PI/4.0, -PI/4.0, 0, 0, -PI/2.0};

    // float** result = kin.forward(joint_angles);
    // utils.print_mat(result, 4);
}

void loop() {
// put your main code here, to run repeatedly:

}
