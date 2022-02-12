#include "Kinematics.h"

Kinematics::Kinematics(int num_of_joints_) {
    num_of_joints = num_of_joints_;
    num_of_joints_declared = 0;
    
    joint_screw_axes = new float*[num_of_joints];
    
    for (int i = 0; i < num_of_joints; i++) {
        joint_screw_axes[i] = new float[6];
    }

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
    end_effector_pose[0][0] = m11;
    end_effector_pose[0][1] = m12;
    end_effector_pose[0][2] = m13;
    end_effector_pose[0][3] = m14;

    end_effector_pose[1][0] = m21;
    end_effector_pose[1][1] = m22;
    end_effector_pose[1][2] = m23;
    end_effector_pose[1][3] = m24;

    end_effector_pose[2][0] = m31;
    end_effector_pose[2][1] = m32;
    end_effector_pose[2][2] = m33;
    end_effector_pose[2][3] = m34;
    
    end_effector_pose[3][0] = m41;
    end_effector_pose[3][1] = m42;
    end_effector_pose[3][2] = m43;
    end_effector_pose[3][3] = m44;
}

float** Kinematics::forward(float* joint_angles) {
    float** transform = utils.create_mat(4);
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transform[i][j] = end_effector_pose[i][j];
        }
    }
    
    for (int i = num_of_joints - 1; i >= 0; i--) {
        transform = utils.mul_matrix(utils.exp6(utils.vec_to_se3(utils.mul_scalar(joint_screw_axes[i], joint_angles[i], 6))), transform, 4);
    }

    return transform;
}

float* Kinematics::inverse(float** T, float* thetalist0, float eomg, float ev) {
    float* thetalist = thetalist0;
    int i = 0;
    int maxiterations = 20;
    float** Tsb = forward(thetalist);
    float* Vs = utils.mul_vector(utils.adjoint(Tsb), utils.se3_to_vec(utils.log6(utils.mul_matrix(utils.trn_mat_inverse(Tsb), T, 4))), 6);
    
    float* w = utils.create_vec(3);
    w[0] = Vs[0];
    w[1] = Vs[1];
    w[2] = Vs[2];
    float* v = utils.create_vec(3);
    v[0] = Vs[0];
    v[1] = Vs[1];
    v[2] = Vs[2];

    while (((utils.norm(w) > eomg) || (utils.norm(v) > ev)) && (i < maxiterations)) {
        thetalist = utils.add_vector(thetalist, utils.mul_vector(utils.pseudo_inverse(jacobian(thetalist), 0, 0), Vs, 6), 6);
        i = i + 1;
        Tsb = forward(thetalist);
        Vs = utils.mul_vector(utils.adjoint(Tsb), utils.se3_to_vec(utils.log6(utils.mul_matrix(utils.trn_mat_inverse(Tsb), T, 4))), 6);
        
        w = utils.create_vec(3);
        w[0] = Vs[0];
        w[1] = Vs[1];
        w[2] = Vs[2];
        v = utils.create_vec(3);
        v[0] = Vs[0];
        v[1] = Vs[1];
        v[2] = Vs[2];
    }

}

float** Kinematics::jacobian(float* joint_angles) {
    float** result = new float*[6];
    for (int i = 0; i < 6; i++) {
        result[i] = new float[num_of_joints];
        for (int j = 0; j < num_of_joints; j++) {
            result[i][j] = joint_screw_axes[j][i];
        }
    }

    float** transform = utils.create_mat(4);

    for (int i = 1; i < num_of_joints; i++) {
        transform = utils.mul_matrix(transform, utils.exp6(utils.vec_to_se3(utils.mul_scalar(joint_screw_axes[i-1], joint_angles[i-1], 6))), 4);
        float* jacobian_column = utils.mul_vector(utils.adjoint(transform), joint_screw_axes[i], 6);
        for (int j = 0; j < 6; j++) {
            result[j][i] = jacobian_column[j];
        }
    }

    return result;
}
