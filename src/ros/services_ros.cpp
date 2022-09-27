#include <UBLOX/ublox_ros.h>
#include "rclcpp/rclcpp.hpp" 

namespace ublox_ros
{
    void UBLOX_ROS::advertiseServices()
    {
        cfg_val_get_ = this->create_service<ublox_read_2::srv::CfgValGet>("CfgValGet", this);
        cfg_val_get_all_ = this->create_service<ublox_read_2::srv::CfgValGetAll>("CfgValGetAll", this);
        cfg_val_del_ = this->create_service<ublox_read_2::srv::CfgValDel>("CfgValDel", this);
        cfg_val_set_ = this->create_service<ublox_read_2::srv::CfgValSet>("CfgValSet", this);
        cfg_reset_ = this->create_service<ublox_read_2::srv::CfgReset>("CfgReset", this);
        init_module_ = this->create_service<ublox_read_2::srv::InitModule>("InitModule", this);
        get_version_ = this->create_service<ublox_read_2::srv::GetVersion>("GetVersion", this);
    }

    bool UBLOX_ROS::getVersion(ublox_read_2::srv::GetVersion::Request &req, ublox_read_2::srv::GetVersion::Response &res)
    {
        ublox::MON_VER_t mon_ver = ublox_->getVersion();

        for(uint8_t i=0; i<30 && mon_ver.swVersion[i]!='\0'; i++)
        {
            res.software_version.push_back(mon_ver.swVersion[i]);
        }

        for(int i=0; i<30 && mon_ver.hwVersion[i]!='\0'; i++)
        {
            res.hardware_version.push_back(mon_ver.hwVersion[i]);
        }

        for(uint8_t i=0; i<10 && mon_ver.extension[i][0]!='\0'; i++)
        {
            std::string extend;
            for(uint8_t j=0; j<30 && mon_ver.extension[i][j]!='\0'; j++)
            {
                std::cerr<<mon_ver.extension[i][j];
                extend.push_back(mon_ver.extension[i][j]);
            }
            res.extension.push_back(extend);
        }
        return true;
    }
    
    bool UBLOX_ROS::cfgValGet(ublox_read_2::srv::CfgValGet::Request &req, ublox_read_2::srv::CfgValGet::Response &res)
    {
        ublox::CFG_VALGET_TUPLE_t response = ublox_->cfgValGet(req.key, req.layer, req.position, req.filepath);
        std::vector<ublox::CFG_VALGET_t::response_t> cfgVector_ublox = std::get<1>(response);
        for(int i=0; i<cfgVector_ublox.size(); i++)
        {
            ublox_read_2::msg::CfgValGetType cfg_ros;
            cfg_ros.version = cfgVector_ublox[i].version;
            cfg_ros.layer = cfgVector_ublox[i].layer;
            cfg_ros.position = cfgVector_ublox[i].position.position;
            cfg_ros.keyID = cfgVector_ublox[i].cfgDataKey.keyID;
            cfg_ros.keyName = std::string(cfgVector_ublox[i].keyName);
            cfg_ros.data = cfgVector_ublox[i].cfgData.data;
            
            res.cfgData.push_back(cfg_ros);
        }
        res.ack=std::get<0>(response).got_ack;
        res.nack=std::get<0>(response).got_nack;
        res.gotcfg=std::get<0>(response).got_cfg_val;
        res.flags=std::get<0>(response).flags;

        return true;
    }

    bool UBLOX_ROS::cfgValGetAll(ublox_read_2::srv::CfgValGetAll::Request &req, ublox_read_2::srv::CfgValGetAll::Response &res)
    {
        ublox::CFG_VALGET_TUPLE_t response = ublox_->cfgValGet(0x0fff0000, req.layer, req.position, req.filepath);
        std::vector<ublox::CFG_VALGET_t::response_t> cfgVector_ublox = std::get<1>(response);
        for(int i=0; i<cfgVector_ublox.size(); i++)
        {
            ublox_read_2::srv::CfgValGetType cfg_ros;
            cfg_ros.version = cfgVector_ublox[i].version;
            cfg_ros.layer = cfgVector_ublox[i].layer;
            cfg_ros.position = cfgVector_ublox[i].position.position;
            cfg_ros.keyID = cfgVector_ublox[i].cfgDataKey.keyID;
            cfg_ros.keyName = std::string(cfgVector_ublox[i].keyName);
            cfg_ros.data = cfgVector_ublox[i].cfgData.data;
            
            res.cfgData.push_back(cfg_ros);
        }
        res.ack=std::get<0>(response).got_ack;
        res.nack=std::get<0>(response).got_nack;
        res.gotcfg=std::get<0>(response).got_cfg_val;
        res.flags=std::get<0>(response).flags;

        return true;
    }

    bool UBLOX_ROS::cfgValDel(ublox_read_2::srv::CfgValDel::Request &req, ublox_read_2::srv::CfgValDel::Response &res)
    {

        ublox::CFG_VAL_DBG_t response = ublox_->cfgValDel(0, req.layer, req.key);

        res.got_Ack = response.got_ack;
        res.got_Nack = response.got_nack;

        return true;
    }



    bool UBLOX_ROS::cfgValSet(ublox_read_2::srv::CfgValSet::Request &req, ublox_read_2::srv::CfgValSet::Response &res)
    {
        ublox::CFG_VAL_DBG_t response = ublox_->cfgValSet(0, req.layer, req.cfgData, req.key);

        res.got_Ack = response.got_ack;
        res.got_Nack = response.got_nack;

        return true;
    }

    bool UBLOX_ROS::cfgReset(ublox_read_2::srv::CfgReset::Request &req, ublox_read_2::srv::CfgReset::Response &res)
    {
        ublox::navBbrMask_t bitfield =  ublox_->reset(req.navBbrMask, req.resetMode);

        // std::cerr<<"eph: "<< bitfield.eph<<std::endl;

        res.eph = bitfield.eph;
        res.alm = bitfield.alm;
        res.health = bitfield.health;
        res.klob = bitfield.klob;
        res.pos = bitfield.pos;
        res.clkd = bitfield.clkd;
        res.osc = bitfield.osc;
        res.utc = bitfield.utc;
        res.rtc = bitfield.rtc;
        res.aop = bitfield.aop;

        return true;

    }

    bool UBLOX_ROS::initModule(ublox_read_2::srv::initModule::Request &req, ublox_read_2::srv::initModule::Response &res)
    {
        switch(req.type)
        {
            case 0:
                initBase();
                break;
            case 1:
                initRover();
                break;
            case 2:
                initBrover();
                break;
            default:
                std::cerr<<"Error: initModule invalid type\n";
                return false;
                break;
        }
        return true;
    }

}