#include <iostream>
#include <fstream>
#include <signal.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;


#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{

UBLOX_ROS::UBLOX_ROS() :
    nh_(), nh_private_("~")
{

    // Connect ROS topics
    advertiseTopics();

    // Connect ROS services
    advertiseServices();

    //Get the serial port
    serial_port_ = nh_private_.param<std::string>("serial_port", "/dev/ttyACM0");
    log_filename_ = nh_private_.param<std::string>("log_filename", "");
    message_rate_ = nh_private_.param<int>("message_rate", 10); //rate at which GNSS measurements are taken in hz
    rover_quantity_ = nh_private_.param<int>("rover_quantity", 0);
    
    // Get Constallation settings
    gps_ = nh_private_.param<int>("GPS", 1); //GPS
    glonas_ = nh_private_.param<int>("GLONAS", 0); //GLONAS
    beidou_ = nh_private_.param<int>("BEIDOU", 0); //BEIDOU
    galileo_ = nh_private_.param<int>("GALILEO", 1); //GALILEO
    dynamic_model_ = nh_private_.param<int>("dynamic_model", 0);
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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ublox_ros");

    ublox_ros::UBLOX_ROS Thing;

    ros::spin();
}
