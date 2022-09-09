#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{
    void UBLOX_ROS::advertiseServices()
    {
        cfg_val_get_ = nh_.advertiseService("CfgValGet", &UBLOX_ROS::cfgValGet, this);
        cfg_val_get_all_ = nh_.advertiseService("CfgValGetAll", &UBLOX_ROS::cfgValGetAll, this);
        cfg_val_del_ = nh_.advertiseService("CfgValDel", &UBLOX_ROS::cfgValDel, this);
        cfg_val_set_ = nh_.advertiseService("CfgValSet", &UBLOX_ROS::cfgValSet, this);
        cfg_reset_ = nh_.advertiseService("CfgReset", &UBLOX_ROS::cfgReset, this);
        init_module_ = nh_.advertiseService("InitModule", &UBLOX_ROS::initModule, this);
        get_version_ = nh_.advertiseService("GetVersion", &UBLOX_ROS::getVersion, this);
    }

    bool UBLOX_ROS::getVersion(ublox::GetVersion::Request &req, ublox::GetVersion::Response &res)
    {
        ublox::MON_VER_t mon_ver = ublox_->getVersion();

        for(uint8_t i=0; i<30 && mon_ver.swVersion[i]!='\0'; i++)
        {
            res.softwareVersion.push_back(mon_ver.swVersion[i]);
        }

        for(int i=0; i<30 && mon_ver.hwVersion[i]!='\0'; i++)
        {
            res.hardwareVersion.push_back(mon_ver.hwVersion[i]);
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
    
    bool UBLOX_ROS::cfgValGet(ublox::CfgValGet::Request &req, ublox::CfgValGet::Response &res)
    {
        ublox::CFG_VALGET_TUPLE_t response = ublox_->cfgValGet(req.key, req.layer, req.position, req.filepath);
        std::vector<ublox::CFG_VALGET_t::response_t> cfgVector_ublox = std::get<1>(response);
        for(int i=0; i<cfgVector_ublox.size(); i++)
        {
            ublox::CfgValGetType cfg_ros;
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

    bool UBLOX_ROS::cfgValGetAll(ublox::CfgValGetAll::Request &req, ublox::CfgValGetAll::Response &res)
    {
        ublox::CFG_VALGET_TUPLE_t response = ublox_->cfgValGet(0x0fff0000, req.layer, req.position, req.filepath);
        std::vector<ublox::CFG_VALGET_t::response_t> cfgVector_ublox = std::get<1>(response);
        for(int i=0; i<cfgVector_ublox.size(); i++)
        {
            ublox::CfgValGetType cfg_ros;
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

    bool UBLOX_ROS::cfgValDel(ublox::CfgValDel::Request &req, ublox::CfgValDel::Response &res)
    {

        ublox::CFG_VAL_DBG_t response = ublox_->cfgValDel(0, req.layer, req.key);

        res.got_Ack = response.got_ack;
        res.got_Nack = response.got_nack;

        return true;
    }



    bool UBLOX_ROS::cfgValSet(ublox::CfgValSet::Request &req, ublox::CfgValSet::Response &res)
    {
        ublox::CFG_VAL_DBG_t response = ublox_->cfgValSet(0, req.layer, req.cfgData, req.key);

        res.got_Ack = response.got_ack;
        res.got_Nack = response.got_nack;

        return true;
    }

    bool UBLOX_ROS::cfgReset(ublox::CfgReset::Request &req, ublox::CfgReset::Response &res)
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

    bool UBLOX_ROS::initModule(ublox::initModule::Request &req, ublox::initModule::Response &res)
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