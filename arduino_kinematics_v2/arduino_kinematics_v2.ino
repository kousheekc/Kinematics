#include "Kinematics.h"
#include "Utilities.h"

void setup() {
// put your setup code here, to run once:
    Serial.begin(9600);

    Kinematics kin(3);
    Utilities utils;

    kin.add_joint_axis(0, 0, 1, 4, 0, 0);
    kin.add_joint_axis(0, 0, 0, 0, 1, 0);
    kin.add_joint_axis(0, 0, -1, -6, 0, -0.1);

    kin.add_initial_end_effector_pose(-1, 0, 0, 0,
                                      0, 1, 0, 6,
                                      0, 0, -1, 2,
                                      0, 0, 0, 1);

    float** T = utils.create_mat(4);
    T[0][0] = 0;
    T[0][1] = 1;
    T[0][2] = 0;
    T[0][3] = -5;

    T[1][0] = 1;
    T[1][1] = 0;
    T[1][2] = 0;
    T[1][3] = 4;

    T[2][0] = 0;
    T[2][1] = 0;
    T[2][2] = -1;
    T[2][3] = 1.6858;

    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;

    float thetalist0[3] = {1.5, 2.5, 3}; 

    kin.inverse(T, thetalist0, 0.01, 0.001);
    
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
