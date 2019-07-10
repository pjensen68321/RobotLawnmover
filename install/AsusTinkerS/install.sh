# Use armbian bionic

# Setup using sudo armbian-config
# i2c0 and i2c4 should be active
# use armbian-config to apt update and upgrade

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-ros-base

sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
cd RobotLawnmover/ROS
catkin_make

echo "source ~/RobotLawnmover/ROS/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc




