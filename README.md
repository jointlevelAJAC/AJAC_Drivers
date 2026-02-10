<!-- GETTING STARTED -->
## Getting Started
This project include (1) Actuator Drivers used by AJAC. (2) USB2FDCAN Drivers used by AJAC. (3) Joint-level QP-kinematics and PMSM Actuators Matlab Simulink Model. (4) AJAC, providing a unified framework that acheives from kinmetic reasoning to actuator realization. 
* *Actuator driver* is developed based on [MIT CHEETAH Actuator Code](https://github.com/mit-biomimetics/Cheetah-Software) and [TI MOTOR Forum](https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum).
* *USB2FDCAN driver* supports 18 actuators working simultaneously with amazing 4ms communication delays.
* *Matlab Simulink* provide a theoretical guide for real motor performance, including parameter tuning (file: scan_the_damping_fator.m) and joint-level kinematic reasoning.
* *AJAC* provide real-time and low-load drivers communicating with USB2FDCAN, including the AJAC (the contract between kinematic reasoning and actuator realization).

## Prerequisites
This project relies on following installations:
* [Eigen 3.4.0](https://gitlab.com/libeigen/eigen/-/releases/3.4.0)
* [LCM 1.5.0](https://github.com/lcm-proj/lcm)
* [libusb](https://libusb.info/)
* [Segger Embedded Studio](https://www.segger.com/products/development-tools/embedded-studio/)
* [CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
* Matlab 2024

