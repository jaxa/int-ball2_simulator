# Install

## Requirements
- **Operating System**: Ubuntu 24.04 Noble
- **ROS Version**: ROS 2 Jazzy
- **Gazebo Version**: Gazebo Harmonic (gz-sim 8)

Additional libraries:

| Name | Version |
| ---- | ---- |
|Qt|5.15 (system)|
|NASM|2.15.05|
|FFmpeg|4.1.3|
|VLC|3.0.7.1|

## 4. Installation
## 4.2 Installing OS
The Int-Ball2 Technology Demonstration Platform runs on Ubuntu 24.04.
To install Ubuntu 24.04 in your environment, refer to the following website:
- [Ubuntu 24.04 LTS (Noble Numbat)](https://releases.ubuntu.com/24.04/)

## 4.3 Installing ROS 2 and Gazebo

The Int-Ball2 Technology Demonstration Platform and its simulator require ROS 2 Jazzy and Gazebo Harmonic.

1. **Install basic tools:**
    ```sh
    sudo apt update
    sudo apt upgrade
    sudo apt install -y wget git vim curl gnupg2 lsb-release software-properties-common
    ```

2. **Set up the ROS 2 repository:** Follow the instructions at:
    - [ROS 2 Jazzy Installation (Ubuntu)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

    ```sh
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    ```

3. **Install ROS 2 Jazzy Desktop:**
    ```sh
    sudo apt install ros-jazzy-desktop
    ```

4. **Install Gazebo Harmonic integration packages:**
    ```sh
    sudo apt install ros-jazzy-ros-gz
    ```

5. **Install additional ROS 2 packages:**
    ```sh
    sudo apt install ros-jazzy-rviz2 \
                     ros-jazzy-tf2-ros \
                     ros-jazzy-tf2-geometry-msgs \
                     ros-jazzy-pcl-ros
    ```

6. **Install colcon build tools:**
    ```sh
    sudo apt install python3-colcon-common-extensions
    ```

7. **Source the ROS 2 setup file:** Add this to your `~/.bashrc`:
    ```sh
    source /opt/ros/jazzy/setup.bash
    ```

## 4.4 Installing Python

This system uses Python 3.11 via pyenv.

1. **Install pyenv dependencies:**
    ```sh
    sudo apt install -y make build-essential libssl-dev zlib1g-dev \
        libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
        libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev \
        liblzma-dev
    ```

2. **Install pyenv:**
    ```sh
    curl https://pyenv.run | bash
    ```
    Add the following to `~/.bashrc`:
    ```sh
    export PYENV_ROOT="$HOME/.pyenv"
    export PATH="$PYENV_ROOT/bin:$PATH"
    eval "$(pyenv init -)"
    ```

3. **Install Python 3.11:**
    ```sh
    pyenv install 3.11
    pyenv global 3.11
    ```

4. **Set LD_LIBRARY_PATH:** Add the following to `~/.bashrc` to ensure Python shared libraries are found:
    ```sh
    export LD_LIBRARY_PATH="$HOME/.pyenv/versions/3.11.13/lib:$LD_LIBRARY_PATH"
    ```

5. **Install Python packages:**
    ```sh
    pip install docker defusedxml netifaces numpy
    ```

## 4.5 Ground Support Equipment
The software for the Technology Demonstration Platform is executed using the Ground Support Equipment (GSE) of Int-Ball2.
Prepare the Int-Ball2 GSE operating environment with the following steps:

### 4.5.1 Netwide Assembler (NASM)
Build and install the assembler called "NASM" from the source file using the following procedure:

1. **Decompress and place the NASM source files under "/usr/local/src":**
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

    **Note:** As mentioned in the reference description [here](https://trans-it.net/centos7-ffmpeg43-h264-fdkaac/), it is necessary to install "nasm" beforehand to install ffmpeg.

### 4.5.2 Video Reception Environment
Build "x264" from the source file using the following steps:

1. **Decompress and place the source file under "/usr/local/src":**

    ```
    cd /usr/local/src
    ```

    (Option1)
    ```sh
    sudo tar jxvf x264-master.tar.bz2 -C /usr/local/src/
    ```

    (Option2)
    ```sh
    sudo git clone https://code.videolan.org/videolan/x264.git /usr/local/src/x264-master
    ```


1. **Move to the directory where the source file is placed and configure the build:**
    ```sh
    cd /usr/local/src/x264-master
    sudo ./configure --disable-asm --enable-shared --enable-static --enable-pic
    ```

2. **Build the source file and install it:**
    ```sh
    sudo make install
    ```

To build `ffmpeg` from the source file, follow these steps:

4. **Decompress and place the `ffmpeg` source file under "/usr/local/src":**
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

2. **Decompress and place the downloaded source file under "/usr/local/src":**

    ```sh
    sudo wget https://download.videolan.org/vlc/3.0.7.1/vlc-3.0.7.1.tar.xz
    sudo tar Jxvf vlc-3.0.7.1.tar.xz -C /usr/local/src/
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
On Ubuntu 24.04, Qt 5.15 is available via the system package manager:

```sh
sudo apt install qtbase5-dev
```

### 4.5.5 Font File
- Since the Int-Ball2 GSE uses the Roboto font, install the font package:

    ```sh
    sudo apt install fonts-roboto
    ```

#### 4.5.6 Source Code Deployment
Deploy "Int-Ball2_platform_gse" to an arbitrary directory.

### 4.5.7 Parameter Settings
To exchange telemetry and commands between the Int-Ball2 Technology Demonstration Platform Simulator and the GSE, configure the communication setting parameters as follows:

- **Command Transmission Settings:**
  Set the IP address of the computer running this system at `intball2_telecommand_target_ip` in `Int-Ball2_platform_gse/src/ground_system/communication_software/config/params.yml`. Set an arbitrary command transmission port at `intball2_telecommand_target_port`.

- **Telemetry Receiving Settings:**
  Set an arbitrary telemetry receiving port at `intball2_telemetry_receive_port` in `Int-Ball2_platform_gse/src/ground_system/communication_software/config/params.yml`.

## 4.6 Int-Ball2 Technology Demonstration Platform Simulator
Prepare the operating environment for using the Int-Ball2 Technology Demonstration Platform Simulator.

### 4.6.1 Docker
Install "Docker" by following the procedure on the following website:
- [Install Docker Engine on Ubuntu | Docker Docs](https://docs.docker.com/engine/install/ubuntu/)
Follow steps 1 and 2 of "Install using the apt repository".

### 4.6.2 Python
- Install Docker-related packages on Python:
    ```sh
    pip install docker defusedxml netifaces
    ```

### 4.6.3 Container Activation Settings
- Ensure the services for containers are always active:
    ```sh
    sudo systemctl enable docker docker.socket
    sudo systemctl start docker.socket
    ```

- Grant the authority to activate containers to the user running this system. Replace `$USER` with the target username if different:
    ```sh
    # Authority setting
    sudo gpasswd -a $USER docker
    sudo chgrp docker /var/run/docker.sock
    # Restart services for containers
    sudo service docker restart
    ```

- Run the following command as the user executing this system to confirm no errors (e.g., "permission denied") occur. If operating via SSH, disconnect and reconnect the SSH session before proceeding.
    ```sh
    docker ps
    ```

### 4.6.4 Source Code Deployment
Deploy "Int-Ball2_platform_simulator" to an arbitrary directory.
Also, deploy [platform_works](https://github.com/jaxa/int-ball2_platform_works) under the home directory of the user executing this system.

### 4.6.5 Parameter Settings
To exchange telemetry and commands between the Int-Ball2 Technology Demonstration Platform Simulator and the GSE, configure the communication setting parameters in the launch files under `Int-Ball2_platform_simulator/src/flight_software/trans_communication/launch/`.

- **Command Transmission Settings:**
  Set the same value for the telecommand port as the receive port in section 4.5.7.

- **Telemetry Receiving Settings:**
  Set the IP address and port of the OCS (Operation Control Station) as launch arguments (`ocs_host`, `ocs_port`).

### 4.6.6 Docker Settings
The Int-Ball2 Technology Demonstration Platform executes the user program using Docker. Configure the following settings related to the exchange between the host and the container:

1. **IP of the Host Computer:**
   Set the IP address of the computer running this system in the platform manager launch configuration.

2. **Container Workspace in the Host Computer:**
   Set the path to `Int-Ball2_platform_simulator` in the platform manager launch configuration.

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

- After the building is completed, execute the following command and confirm that "ib2_user" appears in the "REPOSITORY":
   ```sh
   docker images ib2_user
   ```

## 4.7 User Program Deployment
- The following outlines the deployment method for the user program used in the Technology Demonstration Platform. Deploy the user program in the following directory:
   ```sh
   [Int-Ball2_platform_simulator_deployment_folder]/Int-Ball2_platform_simulator/src/user/
   ```

The files to be deployed should follow the ROS 2 package format described below:

**Python package example (ament_python):**
```
user001/
├── launch/
│   ├── program001.launch.py
│   └── program002.launch.py
├── user001/
│   ├── __init__.py
│   └── user001_node.py
├── package.xml
├── setup.py
└── setup.cfg
```

**C++ package example (ament_cmake):**
```
user001/
├── launch/
│   ├── program001.launch.py
│   └── program002.launch.py
├── src/
│   └── user001_node.cpp
├── include/
│   └── user001/
│       └── user001_node.hpp
├── package.xml
└── CMakeLists.txt
```

User programs are defined as ROS 2 packages. Refer to the official procedure for preparing a new package:
- [Creating a ROS 2 Package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

## 4.8 Launch Files for User Programs

In ROS 2, launch files use Python syntax instead of XML. Below is a template example:

```python
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    return LaunchDescription([
        # Platform launch parameters: specify which existing functions to use.
        # Nodes corresponding to functions set to False are stopped
        # immediately before the start of the user program.
        SetParameter(name='platform_launch.sensor_fusion', value=False),
        SetParameter(name='platform_launch.slam_wrapper', value=False),
        SetParameter(name='platform_launch.ctl_only', value=True),
        SetParameter(name='platform_launch.fsm', value=True),
        SetParameter(name='platform_launch.camera_left', value=True),
        SetParameter(name='platform_launch.camera_right', value=True),

        Node(
            package='user001',
            executable='user001_node',
            name='user001',
            output='screen',
            parameters=[{
                'custom_parameter_integer': 1,
                'custom_parameter_float': 2.0,
                'custom_parameter_string': 'custom',
                'custom_parameter_boolean': True,
            }],
        ),
    ])
```

The correspondence between the platform launch parameter names and the flight software functions is as follows:
- `sensor_fusion`: Sensor fusion
- `slam_wrapper`: Visual SLAM
- `ctl_only`: Thrust Calculation
- `fsm`: Thrust Allocation

To call the newly prepared program while using GSE, add the necessary information to the `user_package_list.json` in the folder `Int-Ball2_platform_gse/src/ground_system/platform_gui/config` where the GSE configuration files are stored. (It does not search for the user program automatically, so it is necessary to write the user program in the list beforehand.)

## 4.9 Building Procedure

### 4.9.1 Int-Ball2 GSE
```sh
source /opt/ros/jazzy/setup.bash
cd <workspace_directory>
colcon build
source install/setup.bash
```

### 4.9.2 Int-Ball2 Technology Demonstration Platform Simulator
```sh
source /opt/ros/jazzy/setup.bash
cd <workspace_directory>
colcon build
source install/setup.bash
```

**Note:** By default, all packages under the workspace will be built. To build specific packages, use:
```sh
colcon build --packages-select <package_name>
```

## 5. Operation Procedure
The procedure to execute this system is shown below:

1. Execute the following commands in a terminal to activate the Int-Ball2 GSE:
    ```sh
    source /opt/ros/jazzy/setup.bash
    source <workspace_directory>/install/setup.bash
    ros2 launch platform_gui bringup.launch.py
    ```

2. Execute the following commands in a different terminal from step 1 to activate the Int-Ball2 Technology Demonstration Platform Simulator:
    ```sh
    source /opt/ros/jazzy/setup.bash
    source <workspace_directory>/install/setup.bash
    ros2 launch ib2_gazebo sim.launch.py
    ```

    To launch the simulator with flight software (platform_manager, trans_communication, etc.):
    ```sh
    ros2 launch platform_sim_tools simulator_bringup_with_flight_software.launch.py
    ```

    **Available launch arguments:**
    | Argument | Default | Description |
    | ---- | ---- | ---- |
    | `gui` | true | Enable Gazebo GUI |
    | `rviz` | true | Enable RViz visualization |
    | `use_ctl_only` | true | Launch thrust calculation node |
    | `use_fsm` | true | Launch thrust allocation node |

    Example (headless mode):
    ```sh
    ros2 launch ib2_gazebo sim.launch.py gui:=false rviz:=false
    ```

3. The Gazebo simulation starts automatically (no need to press a "Play" button).

4. Press the "Navigation ON" button in the "Operation Type" on the "Platform Command" panel of the Int-Ball2 GSE to activate the navigation function.

5. Set User Node, User Launch File, and User Container in the "User Programming Platform" on the "Platform Command" panel of the Int-Ball2 GSE to the user programming function to be executed and press the "Start" button.
   **Example:**
   - **User Node:** sample_tests
   - **User Launch File:** simple_test.launch.py
   - **User Container:** ib2_user:0.1

6. Set User Logic in the "User Programming Platform" on the "Platform Command" panel of the Int-Ball2 GSE to the user logic to be executed and press the "Start" button.

7. When step 6 is completed, press `Ctrl+C` in terminals 1 and 2 to exit the system.
