# Instruction for setting up Jetson Nano SSH

## First Time Setup
A green LED next to the Micro-USB connector will light as soon as the developer kit powers on. When you boot the first time, the developer kit will take you through some initial setup, including:

* Review and accept NVIDIA Jetson software EULA
* Select system language, keyboard layout, and time zone
* Create username, password, and computer name
* Select APP partition size—it is recommended to use the max size suggested

## Setting up SSH in a Jetson Nano

Once the Wifi is setup, download PuTTy and use the localhost for connecting directly over the same internet. Generally of the form `jetson.local` But will depend on what you set suring flahing the image on board.

Then Note down the IP Adress of the Nano and SSH using IP of the form `192.168.100.105`

Can also be done using a COM Port.

# Installing ROS and Other Required software

## ROS installation 

Use the setup script here named `ros.sh` for installing ROS Melodic. 

## Mavros and other important software

Use the setup script here named `mavros.sh` for installing Mavros, Mavproxy, pymavlink and their dependencies.
