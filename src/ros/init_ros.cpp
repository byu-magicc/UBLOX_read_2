#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{
    void UBLOX_ROS::initBase()
    {
        std::cerr<<"Initializing Base\n";

        //Get base parameters
        std::string base_type = nh_private_.param<std::string>("base_type", "moving");
        int surveytime = nh_private_.param<int>("Surveytime", 120); //Stationary base survey time
        int surveyacc = nh_private_.param<int>("Surveyacc", 500000);  //Stationary base accuracy
        std::cerr << "base_type = " << base_type << "\n";
        std::cerr << "surveytime = " << surveytime << "\n";
        std::cerr << "surveyacc = " << surveyacc << "\n";


        //Initialize local arrays to contain parameters from xml file
        std::string* local_host = new std::string[std::max(1, rover_quantity_)];
        uint16_t* local_port = new uint16_t[std::max(1, rover_quantity_)];

        //Initialize rover arrays to contain parameters from xml file
        std::string* rover_host = new std::string[std::max(1, rover_quantity_)];
        uint16_t* rover_port = new uint16_t[std::max(1, rover_quantity_)];
        
        //Account for the case when no numbers are used for the first rover.
        uint8_t j = 0;
        if(nh_private_.hasParam("local_host")) {
            //The first local host corresponds to the first rover.
            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            
            //First rover
            rover_host[0] = nh_private_.param<std::string>("rover_host", "localhost");
            rover_port[0] = nh_private_.param<int>("rover_port", 16145);

            //Let the program know that we have inputted the first rover.
            j=1;
        }

        for(int i=1+j; i <= rover_quantity_; i++) {
            local_host[i-1] = nh_private_.param<std::string>("local_host"+std::to_string(i), "localhost");
            local_port[i-1] = nh_private_.param<int>("local_port"+std::to_string(i), 16140);
            rover_host[i-1] = nh_private_.param<std::string>("rover_host"+std::to_string(i), "localhost");
            rover_port[i-1] = nh_private_.param<int>("rover_port"+std::to_string(i), 16145);
        }

        ublox_->initBase(local_host, local_port, rover_host, rover_port,
          base_type, rover_quantity_, constellation_, surveytime,
          surveyacc, dynamic_model_);
    }

    void UBLOX_ROS::initRover()
    {
        std::cerr<<"Initializing Rover\n";

        //Initialize local arrays to contain parameters from xml file
        std::string* local_host = new std::string[1];
        uint16_t* local_port = new uint16_t[1];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        if(nh_private_.hasParam("local_host")) {
            std::string test = nh_private_.param<std::string>("local_host", "localhost");
            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            base_host[0] = nh_private_.param<std::string>("base_host", "localhost");
            base_port[0] = nh_private_.param<int>("base_port", 16145);
        }
        else {
          local_host[0] = nh_private_.param<std::string>("local_host1", "localhost");
          local_port[0] = nh_private_.param<int>("local_port1", 16140);
          base_host[0] = nh_private_.param<std::string>("base_host1", "localhost");
          base_port[0] = nh_private_.param<int>("base_port1", 16145);
        }

        ublox_->initRover(local_host[0], local_port[0], base_host[0], base_port[0], constellation_, dynamic_model_);
    }

    void UBLOX_ROS::initBrover()
    {
        std::cerr<<"Initializing Brover\n";

        // Initialize local arrays to contain parameters from xml file
        // The first local_host and local_port correspond to the base.
        std::string* local_host = new std::string[rover_quantity_+1];
        uint16_t* local_port = new uint16_t[rover_quantity_+1];

        //Initialize rover arrays to contain parameters from xml file
        std::string* rover_host = new std::string[rover_quantity_];
        uint16_t* rover_port = new uint16_t[rover_quantity_];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        // Fill base arrays with their single values
        base_host[0] = nh_private_.param<std::string>("base_host", "localhost");
        base_port[0] = nh_private_.param<int>("base_port", 16140);

        uint8_t j = 0;
        if(nh_private_.hasParam("local_host")) {

            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            rover_host[0] = nh_private_.param<std::string>("rover_host", "localhost");
            rover_port[0] = nh_private_.param<int>("rover_port", 16145);
            j=1;
        }

        //Input parameters from xml file into respective arrays
        for(int i=1+j; i <= rover_quantity_; i++) {
            local_host[i-1] = nh_private_.param<std::string>("local_host"+std::to_string(i), "localhost");
            local_port[i-1] = nh_private_.param<int>("local_port"+std::to_string(i), 16140);
            rover_host[i-1] = nh_private_.param<std::string>("rover_host"+std::to_string(i), "localhost");
            rover_port[i-1] = nh_private_.param<int>("rover_port"+std::to_string(i), 16145);
            j = i;
        }

        // Add in extra local host values.
        local_host[j] = nh_private_.param<std::string>("local_host"+std::to_string(j+1), "localhost");
        local_port[j] = nh_private_.param<int>("local_port"+std::to_string(j+1), 16140);

        //Determine whether the brover is moving or stationary?
        std::string base_type = "moving";

        // Initiate the Brover
        ublox_->initBrover(local_host, local_port, base_host, base_port,
           rover_host, rover_port, base_type, rover_quantity_, 
           constellation_, dynamic_model_);
    }
}