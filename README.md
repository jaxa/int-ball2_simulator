<div align="center"><img src="https://github.com/jaxa/int-ball2_simulator/blob/main/docs/image/ib2_mission_emblem.png" width="230"/></div>

# Int-Ball2 Simulator

<p style="display: inline">

  <img src="https://img.shields.io/badge/-Ubuntu_18.04_LTS-555555.svg?style=flat&logo=ubuntu">
  <img src="https://img.shields.io/badge/-ROS1--Melodic-%2322314E?style=flat&logo=ROS&logoColor=white">
  <img src="https://img.shields.io/badge/-Python-F2C63C.svg?logo=python&style=flat">
  <img src="https://img.shields.io/badge/-C++-00599C.svg?logo=cplusplus&style=flat">
  <img src="https://img.shields.io/badge/-Docker-1488C6.svg?logo=docker&style=flat">
  <img src="https://img.shields.io/badge/License-Apache--2.0-60C060.svg?style=flat">
</p>

---

## Overview
<img src="https://github.com/jaxa/int-ball2_simulator/blob/main/docs/image/ib2.png" title="Int-Ball2 on the ISS" width="300" align="right" style="display: inline"/>
**Int-Ball2** (JEM Internal Ball Camera 2 System) is a free-flying camera robot deployed in the ISS Japan Experimental Module (JEM). It is remotely controlled from the ground to capture video images and support astronauts. Additionally, Int-Ball2 can run user-developed software as an extended functionality and can be used as a platform for demonstrating robotic technology in space.  

**This Repository** provides a ROS + Gazebo-based simulator for **Int-Ball2**. It simulates Int-Ball2's behavior in the ISS/JEM environment with user-developed programs.
  


## Key Features
- **ROS/Gazebo Simulator**: Simulates Int-Ball2's behavior in the ISS/JEM environment. It includes a plugin simulating the airflow, sensor data acquisition, and actuator control interfaces for Int-Ball2 (SLAM is not simulated).
- **User Program Interface**: Allows users to add new functionalities (e.g., navigation or control algorithms) within Docker containers.
- **User Program Ground Support Equipment (GSE)**: Simulates the GUI of the GSE for user program deployment in actual operations. Enables telemetry reception and command transmission for realistic operational workflows. User programs can be started or stopped from the GSE. (Note: This is not the GSE for nominal Int-Ball2 video-taking operations.)  

  
<div align="center"><img src="https://github.com/jaxa/int-ball2_simulator/blob/main/docs/image/ib2_sim.gif" align="center"/></div>

## Requirements
- **Operating System**: Ubuntu 18.04 Bionic  
- **ROS Version**: ROS 1 Melody (Python3)
- **Gazebo Version**: Gazebo 9 

Additional libraries:

| Name | Version |
| ---- | ---- |
|NumPy|1.18.2|
|EmPy|3.3.4|
|NASM|2.15.05|
|FFmpeg|4.1.3|
|VLC|3.0.7.1|
|Qt|5.12.3|


## Installation
See [INSTALL.md](https://github.com/jaxa/int-ball2_simulator/blob/main/INSTALL.md) for installation instructions.  
Additional information can be found in the [Int-Ball2 Technology Demonstration Platform User's Manual](https://github.com/jaxa/int-ball2_simulator/blob/main/docs/manual/Int-Ball2%20Technology%20Demonstration%20Platform%20User's%20Manual.pdf).


## Project Structure

```
.
├── Int-Ball2_platform_gse/        # Ground Support Equipment S/W
│   └── ...
├── Int-Ball2_platform_simulator/  # 3D Simulator
│   └── ...
├── docs/ 
│   ├── manual/ 
│   │   ├── Int-Ball2 Technology Demonstration Platform User's Manual.pdf  # Manual
│   │   └── Int-Ball2ユーザプラットフォームマニュアル.pdf                     # Manual(JP)
│   └── ...
├── README.md
└── README_JP.md
```



## Troubleshooting
* Common Issues 
  * (TBD)
* Further Assistance
  If you encounter other issues, please open an Issue on GitHub.



## Contributing
We welcome and appreciate your contributions!
* Pull Requests: Submit fixes or new features. Please open an Issue first if it’s a major change.
<!-- For any contribution guidelines or coding standards, see CONTRIBUTING.md (TBD). -->



## License
Copyright (c) 2023, Japan Aerospace Exploration Agency. All rights reserved.  

The Int-Ball2 simulator and user programming platform is licensed under the Apache License, Version 2.0. 
Software  is distributed on an "AS IS" basis, withouth warranties.
See the [license](https://github.com/jaxa/int-ball2_simulator/blob/main/LICENSE) for details. 



## Referencess
About Int-Ball2
* [Paper] [Int-Ball2: ISS JEM Internal Camera Robot with Increased Degree of Autonomy – Design and Initial Checkout, 2024](https://ieeexplore.ieee.org/document/10688008)
* [Paper] [Int-Ball2: On-Orbit Demonstration of Autonomous Intravehicular Flight and Docking for Image Capturing and Recharging, 2024](https://ieeexplore.ieee.org/document/10813456)
* [Paper] [JEM船内可搬型ビデオカメラシステム実証2号機(Int-Ball2)による撮影作業の自動化, 2022](https://www.jstage.jst.go.jp/article/jsmermd/2022/0/2022_1P1-H07/_article/-char/ja/)
* [Paper] [GNC Design and Orbital Performance Evaluation of ISS Onboard Autonomous Free-Flying Robot Int-Ball2, 2024](https://ieeexplore.ieee.org/document/10802183)
* [Paper] [Complementary Ground Testing Method for Autonomous Flight System of Space Free-Flying Robot, 2024](https://ieeexplore.ieee.org/document/10521401)
* [Paper] [Int-Ball2: Compact High-torque Propulsion System Actively Utilizes Propeller Air Drag Polarity, 2023](https://jaxa.repo.nii.ac.jp/record/2000520/files/SA6000193042.pdf)

Others
* [Web] [ROS Melodic](https://wiki.ros.org/melodic)
* [Web] [Gazebo 9.0.0 Release](https://classic.gazebosim.org/blog/gazebo9)

<div align="center"><img src="https://github.com/jaxa/int-ball2_simulator/blob/main/docs/image/ib2_hw.png" height="200" align="center"/>  
<img src="https://github.com/jaxa/int-ball2_simulator/blob/main/docs/image/ib2_sw.png" height="200" align="center"/></div>

## Future Plans
Stay tuned for updates!



