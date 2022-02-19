#include "Kinematics.h"

Kinematics::Kinematics(int num_of_joints_) {
    num_of_joints = num_of_joints_;
    num_of_joints_declared = 0;

    mat_utils.zero((float*)initial_end_effector_pose, 4, 4);
    mat_utils.zero((float*)joint_screw_axes, 6, 6);
}

void Kinematics::add_joint_axis(float s1, float s2, float s3, float s4, float s5, float s6) {
    joint_screw_axes[num_of_joints_declared][0] = s1;
    joint_screw_axes[num_of_joints_declared][1] = s2;
    joint_screw_axes[num_of_joints_declared][2] = s3;
    joint_screw_axes[num_of_joints_declared][3] = s4;
    joint_screw_axes[num_of_joints_declared][4] = s5;
    joint_screw_axes[num_of_joints_declared][5] = s6;

    num_of_joints_declared++;
}

void Kinematics::add_initial_end_effector_pose(float m11, float m12, float m13, float m14, float m21, float m22, float m23, float m24, float m31, float m32, float m33, float m34, float m41, float m42, float m43, float m44) {
    initial_end_effector_pose[0][0] = m11;
    initial_end_effector_pose[0][1] = m12;
    initial_end_effector_pose[0][2] = m13;
    initial_end_effector_pose[0][3] = m14;

    initial_end_effector_pose[1][0] = m21;
    initial_end_effector_pose[1][1] = m22;
    initial_end_effector_pose[1][2] = m23;
    initial_end_effector_pose[1][3] = m24;

    initial_end_effector_pose[2][0] = m31;
    initial_end_effector_pose[2][1] = m32;
    initial_end_effector_pose[2][2] = m33;
    initial_end_effector_pose[2][3] = m34;
    
    initial_end_effector_pose[3][0] = m41;
    initial_end_effector_pose[3][1] = m42;
    initial_end_effector_pose[3][2] = m43;
    initial_end_effector_pose[3][3] = m44;
}

void Kinematics::forward(float* joint_angles, float* transform) {
    float vec6[6];
    float se3[4][4];
    float exp6[4][4];
    float result[4][4];
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transform[4 * i + j] = initial_end_effector_pose[i][j];
        }
    }

    for (int i = num_of_joints - 1; i >= 0; i--) {
        mat_utils.mul_scalar(joint_screw_axes[i], joint_angles[i], 1, 6, vec6);
        mat_utils.vec_to_se3(vec6, (float*)se3);
        mat_utils.exp6((float*)se3, (float*)exp6);
        mat_utils.mul_matrix((float*)exp6, (float*)transform, 4, 4, 4, 4, (float*)result);
        mat_utils.copy_matrix((float*)result, 4, 4, (float*)transform);
    }
}

void Kinematics::inverse(float* transform, float* initial_joint_angles, float ew, float ev, float max_iterations, float* joint_angles) {
    float Tsb[4][4];
    float Tsb_inv[4][4];
    float Tsb_inv_T[4][4];
    float log6[4][4];
    float vec6[6];
    float adj[6][6];
    float Vs[6];
    float w[3];
    float v[3];
    float jac[6][6];
    float pinv[6][6];
    float pinv_Vs[6];
    bool error;
    int i;

    mat_utils.copy_matrix(initial_joint_angles, 1, 6, joint_angles);
    forward(joint_angles, (float*)Tsb);
    mat_utils.trn_mat_inverse((float*)Tsb, (float*)Tsb_inv);
    mat_utils.mul_matrix((float*)Tsb_inv, (float*)transform, 4, 4, 4, 4, (float*)Tsb_inv_T);
    mat_utils.log6((float*)Tsb_inv_T, (float*)log6);
    mat_utils.se3_to_vec((float*)log6, vec6);
    mat_utils.adjoint((float*)Tsb, (float*)adj);
    mat_utils.mul_vector((float*)adj, vec6, 6, 6, Vs);

    w[0] = Vs[0];
    w[1] = Vs[1];
    w[2] = Vs[2];

    v[0] = Vs[3];
    v[1] = Vs[4];
    v[2] = Vs[5];

    error = (mat_utils.norm(w) > ew) || (mat_utils.norm(v) > ev);
    i = 0;

    while (error || i < max_iterations) {
        jacobian(joint_angles, (float*)jac);
        mat_utils.pseudo_inverse((float*)jac, 6, 6, (float*)pinv);
        mat_utils.mul_vector((float*)pinv, Vs, 6, 6, pinv_Vs);
        mat_utils.add_matrix(joint_angles, pinv_Vs, 1, 6, joint_angles);

        i = i + 1;

        forward(joint_angles, (float*)Tsb);

        mat_utils.trn_mat_inverse((float*)Tsb, (float*)Tsb_inv);
        mat_utils.mul_matrix((float*)Tsb_inv, (float*)transform, 4, 4, 4, 4, (float*)Tsb_inv_T);
        mat_utils.log6((float*)Tsb_inv_T, (float*)log6);
        mat_utils.se3_to_vec((float*)log6, vec6);
        mat_utils.adjoint((float*)Tsb, (float*)adj);
        mat_utils.mul_vector((float*)adj, vec6, 6, 6, Vs);

        w[0] = Vs[0];
        w[1] = Vs[1];
        w[2] = Vs[2];

        v[0] = Vs[3];
        v[1] = Vs[4];
        v[2] = Vs[5];

        error = (mat_utils.norm(w) > ew) || (mat_utils.norm(v) > ev);
    }
}

void Kinematics::jacobian(float* joint_angles, float* jacobian) {
    float transform[4][4];
    float vec6[6];
    float se3[4][4];
    float exp6[4][4];
    float result[4][4];
    float adj[6][6];
    float jacobian_column[6];

    mat_utils.zero((float*)jacobian, 6, 6);

    mat_utils.identity((float*)transform, 4);
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            jacobian[6 * i + j] = joint_screw_axes[j][i];
        }
    }

    for (int i = 1; i < num_of_joints; i++) {
        mat_utils.mul_scalar(joint_screw_axes[i-1], joint_angles[i-1], 1, 6, vec6);
        mat_utils.vec_to_se3(vec6, (float*)se3);
        mat_utils.exp6((float*)se3, (float*)exp6);
        mat_utils.mul_matrix((float*)transform, (float*)exp6, 4, 4, 4, 4, (float*)result);
        mat_utils.copy_matrix((float*)result, 4, 4, (float*)transform);

        mat_utils.adjoint((float*)transform, (float*)adj);
        mat_utils.mul_vector((float*)adj, joint_screw_axes[i], 6, 6, jacobian_column);
        for (int j = 0; j < 6; j++) {
            jacobian[6 * j + i] = jacobian_column[j];
        }
    }
}
