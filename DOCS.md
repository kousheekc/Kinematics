## Kinematics Library Usage Guide

### Kinematics Class
This is the class that most users will be primarily interacting with.

- **Include header file**
```c++
#include "Kinematics.h"
```

- **Create Kinematics object**
- **Parameters**
  - (int) number of joints
```c++
Kinematics kinematics_object(3);
```

- **add_joint_axis** - adds the joint axes of the kinematics chain for further in the package while solving kinematic equations
- **Parameters**
  - (float) 1st - 6th component of joint axis
```c++
kinematics_object.add_joint_axis(0, 0,  1,  4, 0,    0);
kinematics_object.add_joint_axis(0, 0,  0,  0, 1,    0);
kinematics_object.add_joint_axis(0, 0, -1, -6, 0, -0.1);
```

- **add_initial_end_effector_pose** - adds the transformation matrix corresponding to the initial end effector pose when the joint angles are all set to 0
- **Parameters**
  - (float) 1st - 16th elements of the 4x4 transformation matrix (traverse left to right starting from row 1)
```c++
kinematics_object.add_initial_end_effector_pose(-1, 0,  0, 0,
                                                 0, 1,  0, 6,
                                                 0, 0, -1, 2,
                                                 0, 0,  0, 1);
```

- **forward** - computes the pose of the end effector frame given a set of joint angles
- **Parameters**
  - (float*) list of joint angles
  - (float*) placeholder for the 4x4 pose output
```c++
float joint_angles[3] = {PI/2.0, 3, PI};
float transform[4][4];

kinematics_object.forward(joint_angles, (float*)transform);
```

- **inverse** - computes the joint angles of the kinematic chain given a desired end effector pose
- **Parameters**
  - (float*) desired end effector pose
  - (float*) placeholder for Jacobian
  - (float*) placeholder for pseudo inverse
  - (float*) placeholder for pseudo inverse transpose
  - (float*) placeholder for pseudo inverse * pseudo inverse transpose
  - (float*) placeholder for pseudo inverse transpose * pseudo inverse
  - (float*) initial joint angles
  - (float) acceptable error for rotation component
  - (float) acceptable error for position component
  - (float) maximum iterations for Newton Raphson method
  - (float*) placeholder for joint angles output
```c++
float jac[6][3];
float jac_t[6][3];
float AA_t[6][6];
float A_tA[3][3];
float pinv[3][6];

float desired_transform[4][4] = {
    {0, 1,  0,     -5},
    {1, 0,  0,      4},
    {0, 0, -1, 1.6858},
    {0, 0,  0,      1}
};

float joint_angles_0[3] = {1.0, 2.5, 3};
float joint_angles[3];

kinematics_object.inverse((float*)desired_transform, (float*)jac, (float*)pinv, (float*)jac_t, (float*)AA_t, (float*)A_tA, joint_angles_0, 0.01, 0.001, 20, j
oint_angles);
```
- **jacobian** - computes the Jacobian of the system
- **Parameters**
  - (float*) current joint angles
  - (float*) placeholder for Jacobian output
```c++
float joint_angles[3] = {1.0, 2.5, 3};
float jac[6][3];

kinematics_object.jacobian(joint_angles, (float*)jac);
```

### MatrixUtils Class
This class provides functionality for matrix manipulation and standard operations with matrices.

**Note:** the methods from this class are primarily used for the backend computations for the Kinematics class and is therfore not intended for basic users. That said, the class provides some useful functionality for debugging and these methods are described below. The rest are quite self explanatory.

- **Include header file**
```c++
#include "MatrixUtils.h"
```

- **Create MatrixUtils object**
```c++
MatrixUtils mat_utils_object;
```

- **print_matrix** - prints a matrix in the serial monitor
- **Parameters**
  - (float*) matrix that you would like to print
  - (int) number of rows of the matrix
  - (int) number of columns of the matrix
  - (String) message describing the matrix
```c++
float M[4][4] = {
    {0, 1,  0,     -5},
    {1, 0,  0,      4},
    {0, 0, -1, 1.6858},
    {0, 0,  0,      1}
};

mat_utils_object.print_matrix((float*)M, 4, 4, "This is the matrix M");
```
