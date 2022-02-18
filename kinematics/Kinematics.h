#ifndef KINEMATICS_h
#define KINEMATICS_h

#include <Arduino.h>
#include "MatrixUtils.h"

class Kinematics {
    private:
        int num_of_joints;
        int num_of_joints_declared;
        float joint_screw_axes[6][6];
        float initial_end_effector_pose[4][4];
        MatrixUtils mat_utils;

    public:
        Kinematics(int num_of_joints_);

        void add_joint_axis(float s1, float s2, float s3, float s4, float s5, float s6);
        void add_initial_end_effector_pose(float m11, float m12, float m13, float m14, float m21, float m22, float m23, float m24, float m31, float m32, float m33, float m34, float m41, float m42, float m43, float m44);
        
        void forward(float* joint_angles, float* transform);
        void inverse(float* transform, float* initial_joint_angles, float ew, float ev, float max_iterations, float* joint_angles);
        void jacobian(float* joint_angles, float* jacobian);
};

#endif