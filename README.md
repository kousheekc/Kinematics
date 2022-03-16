# Kinematics

## Video Demo
Here is a demo of the kinematics library being used as an inverse kinematic controller for a 3D printed 6DOF micro manipulator arm. The arm consists of 6 servo motors and an Arduino Due microcontroller which runs the IK controller using the Kinematics library and sets the joint angles of the servo motors using PWM signals. 

Click on the image below to watch a video which shows the controller jogging the end effector in the cartesian space.

[![VIDEO](http://img.youtube.com/vi/zitple-wJlo/0.jpg)](http://www.youtube.com/watch?v=zitple-wJlo "Open Source Kinematics Library for Arduino based Microncontrollers")

## About The Project
**Description**: This repository contains a kinematics library for the Arduino platform. The library can be used for various robotics projects to solve kinematics equations such as forward kinematics and inverse kinematics. In addition the library provides tools for matrix manipulation, standard operators for matrices and the computation of the Jacobian which can further be used to solve velocity kinematics or statics and dynamics equations.

**Method**: The library uses the **joint screw axes** of the kinematics chain and the **end effector pose at the zero configuration** to determine the kinematic relation between the joint angles and the end effector pose. To solve inverse kinematics, the library uses the **Newton-Raphson Algorithm** which is an iterative numerical algorithm to find the roots of non linear equations.

Here is a GIF of the kinematics library being used to control a small scale 6 DOF manipulator arm built using servo motors and 3D printed parts.

## Getting Started

### Prerequisites
The Arduino IDE is necessary to run Arduino scripts and program your Arduino board. The IDE can be downloaded at the following link: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)

### Installation
To install the kinematics library on the Arduino IDE go to **Sketch -> Include Library -> Manage Libraries** and search for **Kinematics** in the search bar. Select the latest version and click **Install**.

In case this method doesn't work, use the following steps:
- Download or clone this repository
- On the Arduino IDE navigate to **Sketch -> Include Library -> Add .ZIP Library...**
- Select the repository that you just downloaded and click OK

## Usage
The library consists of 2 primary classes. Note that 2 examples scripts are also provided for forward and inverse kinematics. Refer to the [DOCS](DOCS.md) for more information on the classes and methods that are available.

## License
Distributed under the MIT License. See [LICENSE](LICENSE) for more information.

## Contact
Kousheek Chakraborty - kousheekc@gmail.com

Project Link: [https://github.com/kousheekc/Kinematics](https://github.com/kousheekc/Kinematics)

## Acknowledgments
The library is inspired by the **Modern Robotics** course and the accompanying book by **Kevin M. Lynch** and **Frank C. Park**. Here is a link to the material: [http://hades.mech.northwestern.edu/index.php/Modern_Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
