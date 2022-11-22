# ublox_read_2
A library for parsing UBLOX packets and interfacing with UBLOX gps receivers in ROS2. It has been designed for use with the M8N and F9P GNSS receivers.

This library provides rather basic functionality and is designed to work under a linux environment, however it should probably work in Windows or Mac as it uses the cross-platform async_comm library as the serial interface.

The UBX parsing functionality is abstracted into a library for easy integration in other projects. Example usage is given in the main.cpp file.

Hardware documentation found here Software documentation found here.

Derived from https://github.com/byu-magicc/UBLOX_read which is the ROS1 version of this library.

### Cloning and building as a ROS2 package

1. Go to your ROS2 workspace directory (or create one by following the instructions [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)). Make sure you source the setup file in your workspace.

        $ source <path to your workspace>/install/setup.bash

2. In the src directory of your ROS2 workspace, clone this repository.
   
        $ cd <path to your workspace>/src
        $ git clone https://gitlab.magiccvs.byu.edu/boatlanding/ublox_read_2.git

3. After cloning this repository, run the following commands to include the async_comm submodule:

        $ cd ublox_read_2
        $ git submodule update --init --recursive

4. Run the following commands to build the package in your ROS2 workspace.

        $ cd ../../
        $ colcon build --packages-select ublox_read_2

5. Source your workspace again

        $ source <path to your workspace>/install/setup.bash

You are now ready to use this repository in your ROS2 workspace!

### Example launch files

See rover_launch.xml for an example of launching a rover node.\
See base_launch.xml for an example of launching a base node.\
See example_2_rovers_launch.xml for an example of launching a base node with two rover nodes.

