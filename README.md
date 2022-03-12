# Kinematics

## About The Project
**Description**: This repository contains a kinematics library for the Arduino platform. The library can be used for various robotics projects to solve kinematics equations such as forward kinematics and inverse kinematics. In addition the library provides tools to compute the Jacobian of the system which can further be used to solve velocity kinematics or statics and dynamics equations.

**Method**: The library uses the **joint screw axes** of the kinematics chain and the **end effector pose at the zero configuration** to determine the kinematics relation between the joint angles and the end effector pose. To solve inverse kinematics, the library uses the **Newton-Raphson Algorithm** which is an iterative numerical algorithm to find the roots of non linear equations.

Here is a GIF of the kinematics library being used to control a small scale 6 DOF manipulator arm built using servo motors and 3D printed parts.

## Getting Started
Steps to use the kinematics library in your project:

### Prerequisites
The Arduino IDE is necessary to run Arduino scripts and program your Arduino board. The IDE can be downloaded at the following link: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)

### Installation
To install the kinematics library on the Arduino IDE go to **Sketch -> Include Library -> Manage Libraries** and search for **Kinematics** in the search bar. Select the latest version and click **Install**.

## Usage
The library consists of 2 primary classes. These classes along with the corresponding public methods are described below.

### Kinematics class
```c++
#include "Kinematics.h"

Kinematics kinematics_object(6);
```

## License
Distributed under the MIT License. See [LICENSE](LICENSE) for more information.

## Contact
Kousheek Chakraborty - kousheekc@gmail.com

Project Link: [https://github.com/kousheekc/Kinematics](https://github.com/kousheekc/Kinematics)

## Acknowledgments
The library is inspired by the **Modern Robotics** course and the accompanying book by **Kevin M. Lynch** and **Frank C. Park**. Here is a link to the material: [http://hades.mech.northwestern.edu/index.php/Modern_Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
