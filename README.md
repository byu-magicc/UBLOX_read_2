# ublox_read_2
A library for parsing UBLOX packets and interfacing with UBLOX gps receivers in ROS2. It has been designed for use with the M8N and F9P GNSS receivers.

This library provides rather basic functionality and is designed to work under a linux environment, however it should probably work in Windows or Mac as it uses the cross-platform async_comm library as the serial interface.

The UBX parsing functionality is abstracted into a library for easy integration in other projects. Example usage is given in the main.cpp file.

Hardware documentation found here Software documentation found here.

Derived from https://github.com/byu-magicc/UBLOX_read which is the ROS1 version of this library.

### Including async_comm as a submodule
After cloning this repository, run the following command to include the async_comm submodule:

    $ git submodule update --init --recursive


### Example launch files
See rover_launch.xml for an example of launching a rover node.\
See base_launch.xml for an example of launching a base node.\
See example_2_rovers_launch.xml for an example of launching a base node with two rover nodes.\
