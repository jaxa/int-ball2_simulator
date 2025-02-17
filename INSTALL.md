# Install

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

## 4. Installation 
## 4.2 Installing OS
The Int-Ball2 Technology Demonstration Platform runs on Ubuntu 18.04. 
To install Ubuntu 18.04 in your environment, refer to the following website:
- [Ubuntu 18.04.6 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04.6/)

## 4.3 Installing ROS
The Int-Ball2 Technology Demonstration Platform and its simulator require ROS/Gazebo. Since a Gazebo version higher than 9.0.0 is needed, follow these steps:

1. **Preparation to acquire the latest version, Gazebo 9:** To get the latest version of Gazebo 9, execute steps 1 and 2 of the “Alternative installation: step-by-step” on the following website:
    - [Gazebo: Tutorial: Ubuntu](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) (as of Mar. 28, 2024)

2. **Installing ROS:** Install ROS by following steps 1.1 to 1.6 on the following website:
    - [melodic/Installation/Ubuntu - ROS Wiki](http://wiki.ros.org/melodic/Installation/Ubuntu) (as of Mar. 28, 2024)
    - In step 1.4 “Installation,” execute the command:
    ```sh
    sudo apt install ros-melodic-desktop
    ```

3. **Installing the ROS-related package:** Install the ROS-Gazebo collaboration package by executing the following command:
    ```sh
    sudo apt install ros-melodic-gazebo-*
    ```

## 4.4 Installing Python3
This system uses Python3 and its related packages. Follow these steps to install the necessary packages and adjust the settings file:

1. **Install Python3:**
    ```sh
    sudo apt install python3 python3-pip
    ```

2. **Install the ROS-related Python3 packages:**
    ```sh
    pip3 install rospkg 
    pip3 install empy==3.3.4
    ```

    **Note:** When installing "empy" with `pip3 install rospkg empy`, the latest version (4.1 at the time of writing) is installed. However, the reference website [here](http://soup01.com/ja/2023/12/25/post-9789/) confirms this version has a bug causing the error “AttributeError: ‘module’ object has no attribute ‘RAW_OPT’”. Therefore, it is necessary to install version 3.3.4 specifically.

3. **Switching to Python3 for ROS package (catkin) build:** Modify the settings file to use Python3 for building the ROS package (catkin):
    ```sh
    sudo vi /opt/ros/melodic/etc/catkin/profile.d/1.ros_python_version.sh
    ```
    Add the following line:
    ```sh
    export ROS_PYTHON_VERSION=3
    ```

## 4.5 Ground Support Equipment
The software for the Technology Demonstration Platform is executed using the Ground Support Equipment (GSE) of Int-Ball2.
Prepare the Int-Ball2 GSE operating environment with the following steps:

### 4.5.1 Netwide Assembler (NASM)
Build and install the assembler called “NASM” from the source file using the following procedure:

1. **Decompress and place the NASM source files under “/usr/local/src”:**
    ```sh
    cd /usr/local/src
    sudo wget https://www.nasm.us/pub/nasm/releasebuilds/2.15.05/nasm-2.15.05.tar.gz
    sudo tar zxvf nasm-2.15.05.tar.gz
    ```

2. **Move to the directory where the source file is placed and conduct the build settings:**
    ```sh
    cd nasm-2.15.05
    sudo ./configure
    ```

3. **Build the source file and install it:**
    ```sh
    sudo make install
    ```

    **Note:** As mentioned in the reference description [here](https://trans-it.net/centos7-ffmpeg43-h264-fdkaac/), it is necessary to install “nasm” beforehand to install ffmpeg.

### 4.5.2 Video Reception Environment
Build “x264” from the source file using the following steps:

1. **Decompress and place the source file under “/usr/local/src”:**
   
    (Option1)
    ```sh
    sudo tar jxvf x264-master.tar.bz2 -C /usr/local/src/
    ```

    (Option2)
    ```sh
    sudo git clone https://code.videolan.org/videolan/x264.git /usr/local/src/x264-master
    ```


2. **Move to the directory where the source file is placed and configure the build:**
    ```sh
    cd /usr/local/src/x264-master
    sudo ./configure --disable-asm --enable-shared --enable-static --enable-pic
    ```

3. **Build the source file and install it:**
    ```sh
    sudo make install
    ```

To build `ffmpeg` from the source file, follow these steps:

4. **Decompress and place the `ffmpeg` source file under “/usr/local/src”:**
    (Option1)
    ```sh
    sudo tar xvzf ffmpeg-4.1.3.tar.gz -C /usr/local/src/
    ```

    (Option2)
    ```sh
    sudo git clone https://github.com/FFmpeg/FFmpeg.git -b n4.1.3 /usr/local/src/ffmpeg-4.1.3
    ```

5. **Move to the directory where the source file is placed and configure the build:**
    ```sh
    cd /usr/local/src/ffmpeg-4.1.3
    sudo ./configure --extra-cflags="-I/usr/local/include" \
                     --extra-ldflags="-L/usr/local/lib" \
                     --extra-libs="-lpthread -lm -ldl -lpng" \
                     --enable-pic \
                     --disable-programs \
                     --enable-shared \
                     --enable-gpl \
                     --enable-libx264 \
                     --enable-encoder=png \
                     --enable-version3
    ```

6. **Build the source file and install it:**
    ```sh
    sudo make install
    ```

### 4.5.3 Installing VLC Media Player
Install and build the VLC media player using the following procedure:

1. **Install the dependency packages:**
    ```sh
    sudo apt install libasound2-dev libxcb-shm0-dev libxcb-xv0-dev \
                     libxcb-keysyms1-dev libxcb-randr0-dev libxcb-composite0-dev \
                     lua5.2 lua5.2-dev protobuf-compiler bison libdvbpsi-dev libpulse-dev
    ```

2. **Decompress and place the downloaded source file under “/usr/local/src”:**

    (Option1)
    ```sh
    sudo tar Jxvf vlc-3.0.7.1.tar.xz -C /usr/local/src/
    ```

    (Option2)
    ```sh
    sudo git clone https://github.com/videolan/vlc.git -b 3.0.7.1 /usr/local/src/vlc-3.0.7.1
    ```

3. **Move to the directory where the source file is placed and configure the build settings:**
    ```sh
    cd /usr/local/src/vlc-3.0.7.1
    CFLAGS="-I/usr/local/include" \
    LDFLAGS="-L/usr/local/lib" \
    X264_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    X264_LIBS="-lx264" \
    X26410b_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    X26410b_LIBS="-lx264" \
    AVCODEC_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    AVCODEC_LIBS="-lavformat -lavcodec -lavutil" \
    AVFORMAT_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    AVFORMAT_LIBS="-lavformat -lavcodec -lavutil" \
    sudo ./configure --disable-a52 \
                     --enable-merge-ffmpeg \
                     --enable-x264 \
                     --enable-x26410b \
                     --enable-dvbpsi
    ```

4. **Build the source file and install it:**
    ```sh
    sudo make install
    ```

5. **Prepare the symbolic link to the downloaded source file:**
    ```sh
    sudo ln -s /usr/local/src/vlc-3.0.7.1 /usr/local/src/vlc
    ```

### 4.5.4 Qt
Install "Qt" using the following procedure:

1. **Prepare the installation directory:**
    ```sh
    sudo mkdir /opt/Qt
    ```

2. **Activate the downloaded installer for Linux:** (Account registration is required.)
    ```sh
    wget https://download.qt.io/archive/qt/5.12/5.12.3/qt-opensource-linux-x64-5.12.3.run
    chmod +x qt-opensource-linux-x64-5.12.3.run
    ./qt-opensource-linux-x64-5.12.3.run
    ```

3. **Check the box for “Using Open Source Qt”.**

4. **Specify the installation directory as “/opt/Qt”.**

5. **Select “5.12.3 Desktop gcc 64-bit” as the target version for installation and follow the installer’s instructions.**

6. **After installation is complete, prepare the symbolic link:**
    ```sh
    sudo ln -s /opt/Qt/5.12.3 /opt/Qt/5
    ```

### 4.5.5 Font File
- Since the Int-Ball2 GSE uses the Roboto font, copy the font files to the following path and install them:

    (Option1)
    ```sh
    sudo cp Roboto*.ttf /usr/local/share/fonts/.
    fc-cache -f -v
    ```

    (Option2)
    ```sh
    sudo apt install fonts-roboto
    ```

#### 4.5.6 Source Code Deployment
Deploy “Int-Ball2_platform_gse” to an arbitrary directory.

### 4.5.7 Parameter Settings
To exchange telemetry and commands between the Int-Ball2 Technology Demonstration Platform Simulator and the GSE, configure the communication setting parameters as follows:

- **Command Transmission Settings:**
  Set the IP address of the computer running this system at `intball2_telecommand_target_ip` in `IntBall2_platform_gse/src/ground_system/communication_software/config/params.yml`. Set an arbitrary command transmission port at `intball2_telecommand_target_port`.

- **Telemetry Receiving Settings:**
  Set an arbitrary telemetry receiving port at `intball2_telemetry_receive_port` in `IntBall2_platform_gse/src/ground_system/communication_software/config/params.yml`.

## 4.6 Int-Ball2 Technology Demonstration Platform Simulator
Prepare the operating environment for using the Int-Ball2 Technology Demonstration Platform Simulator.

### 4.6.1 Docker
Install “Docker” by following the procedure on the following website:
- [Install Docker Engine on Ubuntu | Docker Docs](https://docs.docker.com/engine/install/ubuntu/) (as of Mar. 28, 2024)
Follow steps 1 and 2 of “Install using the apt repository”.

### 4.6.2 Python
- Install Docker-related packages on Python3:
    ```sh
    pip3 install docker defusedxml netifaces
    ```

### 4.6.3 Container Activation Settings
- Ensure the services for containers are always active:
    ```sh
    sudo systemctl enable docker docker.socket 
    sudo systemctl start docker.socket 
    ```

- Grant the authority to activate containers to the user running this system. Assuming the target user is "nvidia":
    ```sh
    # Authority setting
    sudo gpasswd -a nvidia docker 
    sudo chgrp docker /var/run/docker.sock 
    # Restart services for containers
    sudo service docker restart 
    ```

- Run the following command as the user executing this system to confirm no errors (e.g., "permission denied") occur. If operating via SSH, disconnect and reconnect the SSH session before proceeding.
    ```sh
    docker ps
    ```

### 4.6.4 Source Code Deployment
Deploy “Int-Ball2_platform_simulator” to an arbitrary directory.
Also, deploy [platform_works](https://github.com/jaxa/int-ball2_platform_works) under the home directory of the user executing this system.

### 4.6.5 Parameter Settings
To exchange telemetry and commands between the Int-Ball2 Technology Demonstration Platform Simulator and the GSE, configure the communication setting parameters as follows:

- **Command Transmission Settings:**
  Set the same value for `intball2_telecommand_target_port` as the receive port in section 4.5.7 in `Int-Ball2_platform_simulator/src/flight_software/trans_communication/launch/bringup.launch`.

- **Telemetry Receiving Settings:**
  Set the IP address of the computer running this system at `ocs_host` in `Int-Ball2_platform_simulator/src/flight_software/trans_communication/launch/bringup.launch`. Set the same value for `intball2_telemetry_receive_port` as in section 

### 4.6.6 Docker Settings
The Int-Ball2 Technology Demonstration Platform executes the user program using Docker. Configure the following settings related to the exchange between the host and the container:

1. **IP of the Host Computer:**
   Set the IP address of the computer running this system at `container_ros_master_uri` in `Int-Ball2_platform_simulator/src/platform_sim/platform_sim_tools/launch/platform_manager_bringup.launch`.

2. **Container Workspace in the Host Computer:**
   Set the path to `Int-Ball2_platform_simulator` as `platform works` at `host_ib2_workspace` in `Int-Ball2_platform_simulator/src/platform_sim/platform_sim_tools/launch/platform_manager_bringup.launch`.

### 4.6.7 Setting up Containers
- Since the Technology Demonstration Platform executes the user program in a container, it is necessary to build the container for executing the User Demonstration Platform. The tag name (`ib2_user`) can be set arbitrarily but must have the prefix "ib2" for containers handled by the flight software.
   ```sh
   cd ~/platform_works/platform_docker/template
   docker build . -t ib2_user:0.1
   ```


- Docker images are not cross-platform compatible and cannot be used on different CPU architectures (e.g., images prepared on Intel/AMD for normal computers cannot be used on ARM for Int-Ball2). To prepare images to run on the actual Int-Ball2, use the extended plug-in Docker buildx as shown in the following command:
   ```sh
   docker buildx build --platform linux/arm64/v8 -t ib2_user:(version) --load
   ```

- After the building is completed, execute the following command and confirm that "ib2_user" appears in the “REPOSITORY”:
   ```sh
   docker images ib2_user
   ```

## 4.7 User Program Deployment
- The following outlines the deployment method for the user program used in the Technology Demonstration Platform. Deploy the user program in the following directory:
   ```sh
   [Int-Ball2_platform_simulator_deployment_folder]/Int-Ball2_platform_simulator/src/user/
   ```

The files to be deployed should follow the format described below:

user001/  
├── launch/  
│   ├── program001.launch  
│   ├── program002.launch  
├── package.xml  
├── CMakeLists.txt  
├── (Others)  

User programs are defined as ROS packages. Refer to the official procedure for preparing a new package:
- [Creating a ROS Package](https://wiki.ros.org/ROS/Tutorials/CreatingPackage)

The following is an example of the ROS package configuration for the Technology Demonstration Platform. In this configuration, it is assumed that the package name is “user001” and the user programs included in the package are “program001.launch” and “program002.launch”. The files to be placed under “launch” must follow the template of “roslaunch” for user programs as described below:

| Directory        | Contents                                                                                                           |
|------------------|--------------------------------------------------------------------------------------------------------------------|
| **user001/**     | This is a package that contains a group of programs for a specific user. It can contain multiple user programs inside. Its directory name should be the same as the package name in the ROS package definition file described below. |
| **launch/**      | Deployment directory for “roslaunch” files. The directory name is fixed as “launch”.                              |
| *program001.launch* | Activation settings for user programs (nodes).                                                                  |
| *program002.launch* | Activation settings for user programs (nodes).                                                                  |
| **package.xml**  | ROS package definition file. The “package name” to be defined must match the directory name “user001”. See the official procedure for how to define it: [package.xml](http://wiki.ros.org/catkin/package.xml).     |
| **CMakeLists.txt** | ROS package building settings file. See the official procedure for how to define it: [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt).      |
| **(Others)**     | Arbitrary source code and various configuration files can be deployed.                                             |

## 4.8 `roslaunch` for User Programs

- A template (example definition) of the `roslaunch` file for the Technology Demonstration Platform is shown below. The `<group ns="platform_launch">` is a mandatory item; other items can be set arbitrarily.

    ```xml
    <?xml version="1.0"?>
    <launch>

    <group ns="platform_launch">
    <!-- If set to false, the target nodes will be terminated when user logic is started. -->
    <param name="sensor_fusion" value="false" />
    <param name="slam_wrapper" value="false" />
    <param name="ctl_only" value="true" />
    <param name="fsm" value="true" />
    <!-- If you want to start up camera_left and camera_right, you need to stop (set false) slam_wrapper. -->
    <param name="camera_left" value="true" />
    <param name="camera_right" value="true" />
    </group>

    <node name="user_template" pkg="user_template" type="user_template.py" output="screen">
    <!-- Parameters to be used in the user's program can be set here -->
    <param name="custom_parameter_integer" value="1" />
    <param name="custom_parameter_float" value="2.0" />
    <param name="custom_parameter_string" value="custom" />
    <param name="custom_parameter_boolean" value="true" />
    </node>

    </launch>
    ```
- Specify whether to use the existing functions on the flight software side in `<group ns="platform_launch">`. The ROS node corresponding to the function defined as "not used" (value="false") is stopped immediately before the start of the user implementation logic.

The correspondence between the item names in `<group ns="platform_launch">` and the flight software functions is as follows:
- `sensor_fusion`: Sensor fusion
- `slam_wrapper`: Visual SLAM
- `ctl_only`: Thrust Calculation
- `fsm`: Thrust Allocation

To call the newly prepared program while using GSE, add the necessary information to the `user_package_list.json` in the folder `Int-Ball2_platform_gse/src/ground_system/platform_gui/config` where the GSE configuration files are stored. (It does not search for the user program automatically, so it is necessary to write the user program in the list beforehand.)

## 4.9 Building Procedure

### 4.9.1 Int-Ball2 GSE
- Execute `catkin_make` in the directory where `Int-Ball2_platform_gse` is deployed.
    ```sh
    source /opt/ros/melodic/setup.bash
    catkin_make
    sudo mkdir /var/log/ground_system && sudo chown $USER:$USER /var/log/ground_system
    ```
### 4.9.2 Int-Ball2 Technology Demonstration Platform
    ```sh
    sudo apt install libpcl-dev ros-melodic-pcl-ros
    catkin_make –DWITH_PCA9685=OFF
    ```

- Execute `catkin_make -DWITH_PCA9685=OFF` in the directory where `Int-Ball2_platform_simulator` is deployed. Note that `-DWITH_PCA9685=OFF` is an option to build the thrust function as a simulated node to operate the inductive control function in an environment where the PWM control board is not connected.

## 5. Operation Procedure
The procedure to execute this system is shown below:

1. Execute the following command in the terminal to activate the Int-Ball2 GSE:
    ```sh
    cd [Int-Ball2 GSE deployment path]
    source devel/setup.bash
    roslaunch platform_gui bringup.launch
    ```

2. Execute the following command in a different terminal from step 1 to activate the Int-Ball2 Technology Demonstration Platform Simulator:
    ```sh
    cd [Int-Ball2 Technology Demonstration Platform Simulator deployment path]
    source devel/setup.bash
    rosrun platform_sim_tools simulator_bringup.sh
    ```

3. Press the “Play” button in Gazebo to start the simulation.

4. Press the “Navigation ON” button in the “Operation Type” on the “Platform Command” panel of the Int-Ball2 GSE to activate the navigation function.

5. Set User Node, User Launch File, and User Container in the “User Programming Platform” on the “Platform Command” panel of the Int-Ball2 GSE to the user programming function to be executed and press the “Start” button.
   **Example:**
   - **User Node:** sample_tests
   - **User Launch File:** simple_test.launch
   - **User Container:** ib2_user:0.1

6. Set User Logic in the “User Programming Platform” on the “Platform Command” panel of the Int-Ball2 GSE to the user logic to be executed and press the “Start” button.

7. When step 6 is completed, press `Ctrl+C` in terminals 1 and 2 to exit the system.

**Note:** At the ISS URDF loading, a segmentation fault may occur when trying to load the `node_1` and `node_2` models. In such cases, comment out the relevant section in `iss.urdf`.


