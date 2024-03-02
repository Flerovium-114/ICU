echo "Installing ROS Melodic"
echo

sudo apt update && sudo ap upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
source ~/.bashrc 
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python3-pip
sudo pip3 install -U rospkg
sudo pip3 install pymavlink
sudo apt-get install python-rospkg
sudo apt install python-rosdep
sudo rosdep init
rosdep update

echo
echo "ROS Installation completed!"
echo
