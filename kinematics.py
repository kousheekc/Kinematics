import numpy as np
from scipy.linalg import expm

class Kinematics:
    def __init__(self, end_effector_transform, screw_axes):
        self.end_effector_transform = end_effector_transform
        self.screw_axes = screw_axes

        self.num_of_joints = len(self.screw_axes)
        self.current_joint_angles = np.zeros(self.num_of_joints)

    def forward(self, joint_angles, relative=True):
        transform = self.end_effector_transform

        for i in range(len(joint_angles) - 1, -1, -1):
            transform = np.dot(expm(self.vec_to_se3(self.screw_axes[i] * joint_angles[i])), transform)

        return transform

    def inverse(self, cartesian_position):
        pass

    def vec_to_se3(self, v):
        return np.r_[np.c_[self.vec_to_so3([v[0], v[1], v[2]]), [v[3], v[4], v[5]]], np.zeros((1, 4))]

    def vec_to_so3(self, v):
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])

                         



# end_effector_transform = np.array([[-1, 0,  0, 0],
#                                    [ 0, 1,  0, 6],
#                                    [ 0, 0, -1, 2],
#                                    [ 0, 0,  0, 1]])

# screw_axes = np.array([[0, 0,  1,  4, 0,    0],
#                        [0, 0,  0,  0, 1,    0],
#                        [0, 0, -1, -6, 0, -0.1]])


end_effector_transform = np.array([[ 0,  0,  1, 374],
                                   [ 0, -1,  0, 0],
                                   [ 1,  0,  0, 630],
                                   [ 0,  0,  0, 1]])

screw_axes = np.array([[0, 0, 1, 0, 0, 0],
                       [0, 1, 0, -290, 0, 0],
                       [0, 1, 0, -560, 0, 0],
                       [1, 0, 0, 0, 630, 0],
                       [0, 1, 0, -302, 0, 630],
                       [1, 0, 0, 0, 630, 0]])


joint_angles = np.array([np.pi/2.0, np.pi/4.0,  -np.pi/4.0, 0, 0, -np.pi/2.0])
# joint_angles = np.array([0, 0, 0, 0, 0, 0])

kin = Kinematics(end_effector_transform, screw_axes)
print(kin.forward(joint_angles).round(2))

# Particle Based IK
# Cyclic Coordinate Descent
# Newton Raphson Method
