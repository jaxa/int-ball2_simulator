#!/bin/bash

# Int-Ball2 Simulator Installation Script
# Native installation script converted from Dockerfile

set -e  # Stop script on error

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
  echo "This script must be run with root privileges (sudo bash install_script.sh)"
  exit 1
fi

# Save current user information
CURRENT_USER=$(logname || echo $SUDO_USER)
USER_HOME=$(eval echo $HOME)

echo "===== Starting Int-Ball2 Simulator Installation ====="
echo "Installation user: $CURRENT_USER"
echo "Home directory: $USER_HOME"

# Gazebo preparation
echo "Setting up Gazebo repository..."
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt-get update

# Install ROS Melodic
echo "Installing ROS Melodic..."
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update
apt-get install -y ros-melodic-desktop
apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Install Gazebo packages
echo "Installing Gazebo packages..."
apt-get install -y ros-melodic-gazebo-*

# ROS Python3 configuration
echo "Configuring ROS for Python3..."
sed -i '/ROS_PYTHON_VERSION/s/^/#/; /ROS_PYTHON_VERSION/a export ROS_PYTHON_VERSION=3' /opt/ros/melodic/etc/catkin/profile.d/1.ros_python_version.sh

# Install Python3 packages
echo "Installing Python3 packages..."
apt-get install -y python3 python3-pip
pip3 install rospkg empy==3.3.4 psutil

# Install Netwide Assembler (NASM)
echo "Installing NASM..."
cd /usr/local/src
wget https://www.nasm.us/pub/nasm/releasebuilds/2.15.05/nasm-2.15.05.tar.gz
tar zxvf nasm-2.15.05.tar.gz
cd nasm-2.15.05
./configure
make install

# Install Video Reception Environment
echo "Installing video reception environment..."
git clone https://code.videolan.org/videolan/x264.git /usr/local/src/x264-master
cd /usr/local/src/x264-master
./configure --disable-asm --enable-shared --enable-static --enable-pic
make install

echo "Installing FFmpeg..."
git clone https://github.com/FFmpeg/FFmpeg.git -b n4.1.3 /usr/local/src/ffmpeg-4.1.3
cd /usr/local/src/ffmpeg-4.1.3
./configure --extra-cflags="-I/usr/local/include" \
            --extra-ldflags="-L/usr/local/lib" \
            --extra-libs="-lpthread -lm -ldl -lpng" \
            --enable-pic \
            --disable-programs \
            --enable-shared \
            --enable-gpl \
            --enable-libx264 \
            --enable-encoder=png \
            --enable-version3
make install -j$(nproc)

# Install VLC Media Player
echo "Installing VLC Media Player..."
apt-get install -y libasound2-dev libxcb-shm0-dev libxcb-xv0-dev \
            libxcb-keysyms1-dev libxcb-randr0-dev libxcb-composite0-dev \
            lua5.2 lua5.2-dev protobuf-compiler bison libdvbpsi-dev libpulse-dev
cd /usr/local/src
wget https://download.videolan.org/vlc/3.0.7.1/vlc-3.0.7.1.tar.xz
tar Jxvf vlc-3.0.7.1.tar.xz
cd /usr/local/src/vlc-3.0.7.1
cp ./share/vlc.appdata.xml.in.in ./share/vlc.appdata.xml
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
./configure --disable-a52 \
            --enable-merge-ffmpeg \
            --enable-x264 \
            --enable-x26410b \
            --enable-dvbpsi
make install -j$(nproc)
ln -s /usr/local/src/vlc-3.0.7.1 /usr/local/src/vlc

# Install Qt
mkdir /opt/Qt
wget https://download.qt.io/archive/qt/5.12/5.12.3/qt-opensource-linux-x64-5.12.3.run
sudo chmod +x qt-opensource-linux-x64-5.12.3.run
./qt-opensource-linux-x64-5.12.3.run
sudo ln -s /opt/Qt/5.12.3 /opt/Qt/5

# Install font files
echo "Installing font files..."
apt-get install -y fonts-roboto

# Install Docker
echo "Installing Docker..."
apt-get update
apt-get install -y init systemd apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io
pip3 install docker defusedxml netifaces

# Download and build Int-Ball2 Simulator
echo "Downloading Int-Ball2 Simulator..."
mkdir -p /home/nvidia
cd /home/nvidia
git clone https://github.com/jaxa/int-ball2_simulator.git IB2

# Update parameters
echo "Adjusting simulator settings..."
# Set ROS master URI
#sed -i 's/<arg name="container_ros_master_uri" default="[^"]*"/<arg name="container_ros_master_uri" default="http:\/\/172.17.0.1:11311"/' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/platform_sim_tools/launch/platform_manager_bringup.launch

# Set user workspace path
sed -i 's#<arg name="host_ib2_workspace" default="[^"]*"#<arg name="host_ib2_workspace" default="'"$USER_HOME"'/IB2/Int-Ball2_platform_simulator"#' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/platform_sim_tools/launch/platform_manager_bringup.launch


# Download platform_works repository
echo "Downloading platform_works repository..."
cd /home/nvidia
git clone https://github.com/jaxa/int-ball2_platform_works.git platform_works

# Build GSE
echo "Building GSE..."
cd /home/nvidia/IB2/Int-Ball2_platform_gse
/bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make"
mkdir -p /var/log/ground_system
chown $CURRENT_USER:$CURRENT_USER /var/log/ground_system

# Build Simulator
echo "Building Simulator..."
apt-get install -y libpcl-dev ros-melodic-pcl-ros
cd /home/nvidia/IB2/Int-Ball2_platform_simulator
/bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make -DWITH_PCA9685=OFF"



echo "===== Int-Ball2 Simulator installation completed ====="
