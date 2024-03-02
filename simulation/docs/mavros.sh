echo "Installing Mavros"
echo
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-pip
sudo apt-get install python-dev
sudo pip install future
sudo apt-get install screen python-wxgtk4.0 python-lxml
sudo pip install pyserial
sudo pip install dronekit
sudo pip install MAVProxy
sudo sudo pip install -U pymavlink MAVProxy
sudo apt-get install libcanberra-gtk-module
sudo apt-get install git
sudo apt-get install gitk git-gui
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
sudo wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
sudo apt-get install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-robot-plugins
sudo apt-get install python-catkin-tools
echo
echo "Mavros and Mavproxy installation completed!"
echo
