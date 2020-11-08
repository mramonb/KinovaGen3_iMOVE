# KinovaGen3_iMOVE
This repository presents 3 implementations in C++ for controlling the robotic arm of Kinovarobotics: Kinova Gen3 Ultralight weight with 7 DoF and a C++ code for its compensation of gravity and Coriolis.

This repository encompasses the resulting code of a Master's Thesis (from Master's degree in Robotics and Automatic Control) at UPC (Polytechnic University of Catalonia) in collaboration with IRI (Institut de Robòtica i Informàtica Industrial).

It includes:
- 3 codes for controlling the robotic arm from Kinovarobotics: Kinova Gen3 Ultralight weight with 7 DOF at Low-Level mode (joint space control at 1 kHz)
- 1 code that computes Kinova Gen3 gravity and Coriolis compensation
- 3 codes to test the required drivers or libraries

The required drivers or libraries are the following:
- KINOVA Kortex API (by [Kinovarobotics/kortex](https://github.com/Kinovarobotics/kortex))
- Xsens MVN Awinda (by the IRI driver mvn_xsens for Linux)
- Myo Armband (by [brokenpylons/MyoLinux](https://github.com/brokenpylons/MyoLinux)) 
- Pinocchio: Third-party library to compute Kinova Gen3's dynamics (by [stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio))


