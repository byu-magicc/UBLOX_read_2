#include <iostream>
#include <fstream>
#include <signal.h>

#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/string.h"

#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{

UBLOX_ROS::UBLOX_ROS() : Node("ublox_ros") {

    // Connect ROS topics
    advertiseTopics();

    // Connect ROS services
    advertiseServices();

    //Get the serial port
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<std::string>("log_filename", "");
    this->declare_parameter<int>("message_rate", 10); //rate at which GNSS measurements are taken in hz
    this->declare_parameter<int>("rover_quantity", 0);
    
    this->get_parameter<std::string>("serial_port", serial_port_);
    this->get_parameter<std::string>("log_filename", log_filename_);
    this->get_parameter<int>("message_rate", message_rate_); //rate at which GNSS measurements are taken in hz
    this->get_parameter<int>("rover_quantity", rover_quantity_);
    
    // Get Constallation settings
    this->declare_parameter<int>("GPS", 1);
    this->declare_parameter<int>("GLONAS", 0);
    this->declare_parameter<int>("BEIDOU", 0);
    this->declare_parameter<int>("GALILEO", 1);
    this->declare_parameter<int>("dynamic_model", 0);

    this->get_parameter<std::string>("GPS", gps_);
    this->get_parameter<std::string>("GLONAS", glonas_);
    this->get_parameter<std::string>("BEIDOU", beidou_);
    this->get_parameter<std::string>("GALILEO", galileo_);
    this->get_parameter<std::string>("dynamic_model", dynamic_model_);
    
    std::cerr << "message_rate = " << message_rate_ << "\n";
    std::cerr << "rover_quantity = " << rover_quantity_ << "\n";
    std::cerr << "gps = " << gps_ << "\n";
    std::cerr << "glonas = " << glonas_ << "\n";
    std::cerr << "beidou = " << beidou_ << "\n";
    std::cerr << "galileo = " << galileo_ << "\n";
    constellation_.gps_enable = gps_;
    constellation_.glonas_enable = glonas_;
    constellation_.beidou_enable = beidou_;
    constellation_.galileo_enable = galileo_;

    // create the parser
    ublox_ = new ublox::UBLOX(serial_port_, message_rate_);

    // set up RTK
    // Base (n local_host n local_port, n rover_host, n rover_port)
    

    if(nh_private_.param<bool>("debug", false))
    {
        std::cerr<<"DEBUG MODE\n";
    }
    else if (!nh_private_.hasParam("base_host"))
    {
        initBase();
    }
    // Rover(1 local_host 1 local_port 1 base_host 1 base_port)
    else if (!nh_private_.hasParam("rover_host"))
    {
        initRover();
    }
    // Brover(1 base_host 1 base_port n local_host n local_port n rover_host n rover_port)
    else if (nh_private_.hasParam("base_host") && nh_private_.hasParam("rover_host")) 
    {
        initBrover();
    }
    else
    {
        std::cerr<<"Could not deduce base, rover, or brover\n";
    }
    

    // Check if there is a arrow
    if (nh_private_.hasParam("arrowbase") && nh_private_.hasParam("arrowtip")) {

      // If there is an arrow , then we need to subscribe to the base
      std::string arrowbase = nh_private_.param<std::string>("arrowbase", "/brover");
      // and tip of the arrow for /RelPos
      std::string arrowtip = nh_private_.param<std::string>("arrowtip", "/rover");

      // Call the first subscriber
      sub1 = nh_.subscribe(arrowbase+"/RelPos", 10, &UBLOX_ROS::cb_rov1, this);

      // Call the second subscriber
      sub2 = nh_.subscribe(arrowtip+"/RelPos", 10, &UBLOX_ROS::cb_rov2, this);

      // Make the arrow flag true. This flag is used in the relposCB function in
      // in order to determine if the vector math function needs to be called or
      // not.
      arrow_flag = true;
    }

    // connect callbacks
    createCallback(ublox::CLASS_NAV, ublox::NAV_RELPOSNED, &UBLOX_ROS::relposCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_POSECEF, &UBLOX_ROS::posECEFCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_VELECEF, &UBLOX_ROS::velECEFCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_SVIN, &UBLOX_ROS::svinCB, this);
    createCallback(ublox::CLASS_RXM, ublox::RXM_RAWX, &UBLOX_ROS::obsCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_PVT, &UBLOX_ROS::pvtCB, this);
    createCallback(ublox::CLASS_RXM, ublox::RXM_RTCM, &UBLOX_ROS::rtcmInputCB, this);
    createCallback(ublox::CLASS_RXM, ublox::RXM_MEASX, &UBLOX_ROS::rxmMeasxCB, this);
    if (!log_filename_.empty())
    {
        ublox_->initLogFile(log_filename_);
        //ublox_->readFile(log_filename_);
    }
}

UBLOX_ROS::~UBLOX_ROS()
{
    if (ublox_)
        delete ublox_;
}

bool UBLOX_ROS::evalF9PID(uint8_t f9pID)
{
    switch(f9pID)
    {
        case 0:
            ecef_ptr_= &ecef_msg_;
            ecef_pub_ptr_ = &ecef_pub_;
            pvt_ptr_ = &pvt_msg_;
            pvt_pub_ptr_ = &pvt_pub_;
            ecef_pos_tow_ptr_ = &ecef_pos_tow_;
            ecef_vel_tow_ptr_ = &ecef_vel_tow_;
            pvt_tow_ptr_ = &pvt_tow_;
            return true;
            break;
        case 1:
            ecef_ptr_= &base_ecef_msg_;
            ecef_pub_ptr_ = &base_ecef_pub_;
            pvt_ptr_ = &base_pvt_msg_;
            pvt_pub_ptr_ = &base_pvt_pub_;
            ecef_pos_tow_ptr_ = &base_ecef_pos_tow_;
            ecef_vel_tow_ptr_ = &base_ecef_vel_tow_;
            pvt_tow_ptr_ = &base_pvt_tow_;
            return true;
            break;
        default:
            return false;
            break;
    }
}

}
