# Use armbian bionic

# Setup using sudo armbian-config
# i2c0 and i2c4 should be active
# use armbian-config to apt update and upgrade

# Setup ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-ros-base -Y

sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install dependencies
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential python-dev python-pip i2c-tools -Y

# install GPIO
git clone http://github.com/TinkerBoard/gpio_lib_python --depth 1 GPIO_API_for_Python
git clone http://github.com/TinkerBoard/gpio_lib_c --depth 1 GPIO_API_for_C

cd GPIO_API_for_Python/
sudo python setup.py install
cd ..
cd GPIO_API_for_C/
sudo chmod +x build
sudo ./build

# install i2c smbus
sudo pip install smbus

# Setup robot code
cd RobotLawnmover/ROS
catkin_make

echo "source ~/RobotLawnmover/ROS/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc




