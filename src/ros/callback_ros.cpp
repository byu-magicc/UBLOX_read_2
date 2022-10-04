#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{
    void UBLOX_ROS::advertiseTopics()
    {
        pvt_pub_ = this->create_publisher<ublox_read_2::msg::PositionVelocityTime>("PosVelTime", 10);
        relpos_pub_ = this->create_publisher<ublox_read_2::msg::RelPos>("RelPos", 10);
        relposflag_pub_ = this->create_publisher<ublox_read_2::msg::RelPosFlags>("RelPosFlags", 10);
        ecef_pub_ = this->create_publisher<ublox_read_2::msg::PosVelEcef>("PosVelEcef", 10);
        survey_status_pub_ = this->create_publisher<ublox_read_2::msg::SurveyStatus>("SurveyStatus", 10);
        eph_pub_ = this->create_publisher<ublox_read_2::msg::Ephemeris>("Ephemeris", 10);
        geph_pub_ = this->create_publisher<ublox_read_2::msg::GlonassEphemeris>("GlonassEphemeris", 10);
        obs_pub_ = this->create_publisher<ublox_read_2::msg::ObsVec>("Obs", 10);
        base_ecef_pub_ = this->create_publisher<ublox_read_2::msg::PosVelEcef>("base/PosVelEcef", 10);
        base_pvt_pub_ = this->create_publisher<ublox_read_2::msg::PositionVelocityTime>("base/PosVelTime", 10);
        rtcm_input_pub_ = this->create_publisher<ublox_read_2::msg::RTCMInput>("RTCMInput", 10);
        sat_status_pub_ = this->create_publisher<ublox_read_2::msg::SatelliteStatus>("SatelliteStatus", 10);
        // nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::NavSatFix>("NavSatFix");
        // nav_sat_status_pub_ = this->create_publisher<sensor_msgs::NavSatStatus>("NavSatStatus");
    }
    
    // Callback function for subscriber to RelPos for a given RelPos message.
// NOTE: This message is not the same as ublox::NAV_RELPOSNED_t, since that one
// deals with messages from the f9p
void UBLOX_ROS::cb_rov1(const ublox_read_2::msg::RelPos &msg) {
    ned_1[0] = msg.rel_pos_ned[0];  //North
    ned_1[1] = msg.rel_pos_ned[1];  //East
    ned_1[2] = msg.rel_pos_ned[2];  //Down
}

// Callback function for subscriber to second RelPos.
// NOTE: This message is not the same as ublox::NAV_RELPOSNED_t, since that one
// deals with messages from the f9p
void UBLOX_ROS::cb_rov2(const ublox_read_2::msg::RelPos &msg) {
    ned_2[0] = msg.rel_pos_ned[0];  //North
    ned_2[1] = msg.rel_pos_ned[1];  //East
    ned_2[2] = msg.rel_pos_ned[2];  //Down
}

void UBLOX_ROS::pvtCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_PVT_t msg = ubx_msg.NAV_PVT;

    if(!evalF9PID(f9pID)) return;

    *pvt_tow_ptr_ = msg.iTOW;
    // out.iTOW = msg.iTow;
    pvt_ptr_->header.stamp = this->get_clock().now().to_msg(); ///TODO: Do this right
    pvt_ptr_->year = msg.year;
    pvt_ptr_->month = msg.month;
    pvt_ptr_->day = msg.day;
    pvt_ptr_->hour = msg.hour;
    pvt_ptr_->min = msg.min;
    pvt_ptr_->sec = msg.sec;
    pvt_ptr_->nano = msg.nano;
    pvt_ptr_->tAcc = msg.tAcc;
    pvt_ptr_->valid = msg.valid;
    pvt_ptr_->fixType = msg.fixType;
    pvt_ptr_->flags = msg.flags;
    pvt_ptr_->flags2 = msg.flags2;
    pvt_ptr_->numSV = msg.numSV;
    pvt_ptr_->lla[0] = msg.lat*1e-7;
    pvt_ptr_->lla[1] = msg.lon*1e-7;
    pvt_ptr_->lla[2] = msg.height*1e-3;
    pvt_ptr_->hMSL = msg.hMSL*1e-3;
    pvt_ptr_->hAcc = msg.hAcc*1e-3;
    pvt_ptr_->vAcc = msg.vAcc*1e-3;
    pvt_ptr_->velNED[0] = msg.velN*1e-3;
    pvt_ptr_->velNED[1] = msg.velE*1e-3;
    pvt_ptr_->velNED[2] = msg.velD*1e-3;
    pvt_ptr_->gSpeed = msg.gSpeed*1e-3;
    pvt_ptr_->headMot = msg.headMot*1e-5;
    pvt_ptr_->sAcc = msg.sAcc*1e-3;
    pvt_ptr_->headAcc = msg.headAcc*1e-5;
    pvt_ptr_->pDOP = msg.pDOP*0.01;
    pvt_ptr_->headVeh = msg.headVeh*1e-5;

    pvt_pub_ptr_->publish(*pvt_ptr_);

    ecef_ptr_->header.stamp = ros::Time::now();
    ecef_ptr_->fix = pvt_ptr_->fixType;
    ecef_ptr_->lla[0] = pvt_ptr_->lla[0];
    ecef_ptr_->lla[1] = pvt_ptr_->lla[1];
    ecef_ptr_->lla[2] = pvt_ptr_->lla[2];
    ecef_ptr_->horizontal_accuracy = pvt_ptr_->hAcc;
    ecef_ptr_->vertical_accuracy = pvt_ptr_->vAcc;
    ecef_ptr_->speed_accuracy = pvt_ptr_->sAcc;

    if (*ecef_pos_tow_ptr_ == *pvt_tow_ptr_ && 
        *ecef_pos_tow_ptr_ == *ecef_vel_tow_ptr_)
    {
        ecef_pub_ptr_->publish(*ecef_ptr_);
    }
}


void UBLOX_ROS::relposCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    // std::cerr<<"relposCB"<<std::endl;
    
    ublox::NAV_RELPOSNED_t msg = ubx_msg.NAV_RELPOSNED;
    
    // Create the message to be outputted
    ublox::RelPos out;


    // out.iTOW = msg.iTow*1e-3;
    out.header.stamp = ros::Time::now(); /// TODO: do this right
    out.refStationId = msg.refStationId;
    out.relPosNED[0] = msg.relPosN*1e-2;
    out.relPosNED[1] = msg.relPosE*1e-2;
    out.relPosNED[2] = msg.relPosD*1e-2;
    out.relPosLength = msg.relPosLength*1e-2;
    out.relPosHeading = deg2rad(msg.relPosHeading*1e-5);
    out.relPosHPNED[0] = msg.relPosHPN*1e-3*.1;
    out.relPosHPNED[1] = msg.relPosHPE*1e-3*.1;
    out.relPosHPNED[2] = msg.relPosHPD*1e-3*.1;
    out.relPosHPLength = msg.relPosHPLength*1e-3*.1;
    out.accNED[0] = msg.accN*1e-3*.1;
    out.accNED[1] = msg.accE*1e-3*.1;
    out.accNED[2] = msg.accD*1e-3*.1;
    out.accLength = msg.accLength*1e-3*.1;
    out.accHeading = deg2rad(msg.accHeading*1e-5);
    out.flags = msg.flags.all_flags;


    relpos_flag_msg_.header.stamp = out.header.stamp;
    relpos_flag_msg_.gnssFixOk = msg.flags.gnssFixOk;
    relpos_flag_msg_.diffSoln = msg.flags.diffSoln;
    relpos_flag_msg_.relPosValid = msg.flags.relPosValid;
    relpos_flag_msg_.floatCarrSoln = msg.flags.floatCarrSoln;
    relpos_flag_msg_.fixedCarrSoln = msg.flags.fixedCarrSoln;
    relpos_flag_msg_.isMoving = msg.flags.isMoving;
    relpos_flag_msg_.refPosMiss = msg.flags.refPosMiss;
    relpos_flag_msg_.refObsMiss = msg.flags.refObsMiss;
    relpos_flag_msg_.relPosHeadingValid = msg.flags.relPosHeadingValid;
    relpos_flag_msg_.relPosNormalized = msg.flags.relPosNormalized;
    relpos_flag_msg_.flags = msg.flags.all_flags;

    if (arrow_flag == true) {

    // Perform vector_math and assign values to arrow. (see ublox_ros.h for
    // variable declarations)
    ublox_->vector_math(ned_1, ned_2, arrow);

    // Assign all the values
    out.arrowNED[0] = arrow[0];
    out.arrowNED[1] = arrow[1];
    out.arrowNED[2] = arrow[2];
    out.arrowLength = arrow[3];
    out.arrowRPY[0] = arrow[4];
    out.arrowRPY[1] = arrow[5];
    out.arrowRPY[2] = arrow[6];
  }
    // Publish the RelPos ROS message
    relpos_pub_.publish(out);
    relposflag_pub_.publish(relpos_flag_msg_);
}

void UBLOX_ROS::svinCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_SVIN_t msg = ubx_msg.NAV_SVIN;
    ublox::SurveyStatus out;
    out.header.stamp = ros::Time::now(); /// TODO: do this right
    out.dur = msg.dur;
    out.meanXYZ[0] = msg.meanX*1e-2;
    out.meanXYZ[1] = msg.meanY*1e-2;
    out.meanXYZ[2] = msg.meanZ*1e-2;
    out.meanXYZHP[0] = msg.meanXHP*1e-3;
    out.meanXYZHP[1] = msg.meanYHP*1e-3;
    out.meanXYZHP[2] = msg.meanZHP*1e-3;
    out.meanAcc = msg.meanAcc;
    out.obs = msg.obs;
    out.valid = msg.valid;
    out.active = msg.active;
    survey_status_pub_.publish(out);

}

void UBLOX_ROS::posECEFCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_POSECEF_t msg = ubx_msg.NAV_POSECEF;

    if(!evalF9PID(f9pID)) return;

    *ecef_pos_tow_ptr_ = msg.iTOW;
    ecef_ptr_->header.stamp = ros::Time::now();
    ecef_ptr_->position[0] = msg.ecefX*1e-2;
    ecef_ptr_->position[1] = msg.ecefY*1e-2;
    ecef_ptr_->position[2] = msg.ecefZ*1e-2;

    if (*ecef_pos_tow_ptr_ == *pvt_tow_ptr_ && 
        *ecef_pos_tow_ptr_ == *ecef_vel_tow_ptr_)
    {
        ecef_pub_ptr_->publish(*ecef_ptr_);
    }

}

void UBLOX_ROS::velECEFCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_VELECEF_t msg = ubx_msg.NAV_VELECEF;

    if(!evalF9PID(f9pID)) return;

    *ecef_vel_tow_ptr_ = msg.iTOW;
    ecef_ptr_->header.stamp = ros::Time::now();
    ecef_ptr_->velocity[0] = msg.ecefVX*1e-2;
    ecef_ptr_->velocity[1] = msg.ecefVY*1e-2;
    ecef_ptr_->velocity[2] = msg.ecefVZ*1e-2;

    if (*ecef_pos_tow_ptr_ == *pvt_tow_ptr_ && 
        *ecef_pos_tow_ptr_ == *ecef_vel_tow_ptr_)
    {
        ecef_pub_ptr_->publish(*ecef_ptr_);
    }
}

void UBLOX_ROS::obsCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::RXM_RAWX_t msg = ubx_msg.RXM_RAWX;
    ublox::ObsVec out;
    UTCTime utc =UTCTime::fromGPS(msg.week, msg.rcvTow*1e3);
    out.header.stamp.sec = utc.sec;
    out.header.stamp.nsec = utc.nsec;
    for (int i = 0; i < msg.numMeas; i++)
    {
        out.obs[i].sat = msg.meas[i].svId;
        out.obs[i].gnssID = msg.meas[i].gnssId;
        out.obs[i].signal = ublox::sigId(msg.meas[i].gnssId, msg.meas[i].sigId);
        switch (out.obs[i].signal)
        {
        case ublox::Observation::GPS_L1_CA:
        case ublox::Observation::GALILEO_E1_B:
        case ublox::Observation::GALILEO_E1_C:
        case ublox::Observation::QZSS_L1_CA:
            out.obs[i].freq = Ephemeris::GPS_FREQL1;
            break;
        case ublox::Observation::GPS_L2_CL:
        case ublox::Observation::GPS_L2_CM:
            out.obs[i].freq = Ephemeris::GPS_FREQL2;
            break;
        case ublox::Observation::GLONASS_L1:
            out.obs[i].freq = GlonassEphemeris::FREQ1_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ1_GLO;
            break;
        case ublox::Observation::GLONASS_L2:
            out.obs[i].freq = GlonassEphemeris::FREQ2_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ2_GLO;
            break;
            // These may not be right
//        case ublox::Observation::GALILEO_E5_BI:
//        case ublox::Observation::GALILEO_E5_BQ:
//            out.obs[i].freq = Ephemeris::GALILEO_FREQL5b;
//            break;
//        case ublox::Observation::BEIDOU_B1I_D1:
//        case ublox::Observation::BEIDOU_B1I_D2:
//            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B1;
//            break;
//        case ublox::Observation::BEIDOU_B2I_D1:
//        case ublox::Observation::BEIDOU_B2I_D2:
//            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B2;
//            break;
        default:
            out.obs[i].freq = 0;
            break;
        }
        out.obs[i].cno = msg.meas[i].cno;
        out.obs[i].locktime = msg.meas[i].locktime;
        out.obs[i].P = msg.meas[i].prMeas;
        out.obs[i].L = msg.meas[i].cpMeas;
        out.obs[i].D = msg.meas[i].doMeas;
        out.obs[i].stdevP = 0.01 * pow(2, msg.meas[i].prStdev);
        out.obs[i].stdevL = 0.004 * msg.meas[i].cpStdev;
        out.obs[i].stdevD = 0.002 * pow(2, msg.meas[i].doStdev);

        // indicate cycle slip
        if (msg.meas[i].cpMeas != 0.0
            && (msg.meas[i].trkStat & ublox::RXM_RAWX_t::trkStat_HalfCyc | ublox::RXM_RAWX_t::trkStat_subHalfCyc))
        {
            out.obs[i].LLI =  ublox::Observation::LLI_HALF_CYCLE_AMB;
        }
        else
        {
            out.obs[i].LLI = 0;
        }
    }
    obs_pub_.publish(out);
}

void UBLOX_ROS::ephCB(const Ephemeris &eph)
{
    ublox::Ephemeris out;
    out.header.stamp = ros::Time::now();

    out.sat = eph.sat;
    out.gnssID = eph.gnssID;
    out.toe.sec = eph.toe.sec;
    out.toe.nsec = eph.toe.nsec;
    out.toc.sec = eph.toc.sec;
    std::cerr<<"About to spin\n";
    out.toc.nsec = eph.toc.nsec;

    out.tow = eph.tow;
    out.iodc = eph.iodc;
    out.iode = eph.iode;
    out.week = eph.week;
    out.toes = eph.toes;
    out.tocs = eph.tocs;
    out.health = eph.health;
    out.alert_flag = eph.alert_flag;
    out.anti_spoof = eph.anti_spoof;
    out.code_on_L2 = eph.code_on_L2;
    out.ura = eph.ura;
    out.L2_P_data_flag = eph.L2_P_data_flag;
    out.fit_interval_flag = eph.fit_interval_flag;
    out.age_of_data_offset = eph.age_of_data_offset;
    out.tgd[0] = eph.tgd[0];
    out.tgd[1] = eph.tgd[1];
    out.tgd[2] = eph.tgd[2];
    out.tgd[3] = eph.tgd[3];
    out.af2 = eph.af2;
    out.af1 = eph.af1;
    out.af0 = eph.af0;
    out.m0 = eph.m0;
    out.delta_n = eph.delta_n;
    out.ecc = eph.ecc;
    out.sqrta = eph.sqrta;
    out.omega0 = eph.omega0;
    out.i0 = eph.i0;
    out.w = eph.w;
    out.omegadot = eph.omegadot;
    out.idot = eph.idot;
    out.cuc = eph.cuc;
    out.cus = eph.cus;
    out.crc = eph.crc;
    out.crs = eph.crs;
    out.cic = eph.cic;
    out.cis = eph.cis;

    eph_pub_.publish(out);
}

void UBLOX_ROS::gephCB(const GlonassEphemeris &eph)
{
    ublox::GlonassEphemeris out;
    out.header.stamp = ros::Time::now();

    out.sat = eph.sat;
    out.gnssID = eph.gnssID;

    out.toe.sec = eph.toe.sec;
    out.toe.nsec = eph.toe.nsec;
    out.tof.sec = eph.tof.sec;
    out.tof.nsec = eph.tof.nsec;

    out.iode = eph.iode;
    out.frq = eph.frq;
    out.svh = eph.svh;
    out.sva = eph.sva;
    out.age = eph.age;
    out.pos[0] = eph.pos[0];
    out.pos[1] = eph.pos[1];
    out.pos[2] = eph.pos[2];
    out.vel[0] = eph.vel[0];
    out.vel[1] = eph.vel[1];
    out.vel[2] = eph.vel[2];
    out.acc[0] = eph.acc[0];
    out.acc[1] = eph.acc[1];
    out.acc[2] = eph.acc[2];
    out.taun = eph.taun;
    out.gamn = eph.gamn;
    out.dtaun = eph.dtaun;

    geph_pub_.publish(out);
}

void UBLOX_ROS::rtcmInputCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::RXM_RTCM_t msg = ubx_msg.RXM_RTCM;
    
    ublox::RTCMInput out;
    out.header.stamp = ros::Time::now();
    out.version = msg.version;
    out.flags = msg.flags;
    out.crcFailed = msg.crcFailed;
    out.subType = msg.subType;
    out.refStation = msg.refStation;
    out.msgType = msg.msgType;

    rtcm_input_pub_.publish(out);
}

void UBLOX_ROS::rxmMeasxCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::RXM_MEASX_t msg = ubx_msg.RXM_MEASX;

    ublox::SatelliteStatus out;
    out.version = msg.version;
    out.gpsTOW = msg.gpsTOW;
    out.gloTOW = msg.gloTOW;
    out.bdsTOW = msg.bdsTOW;
    out.qzssTOW = msg.qzssTOW;
    out.gpsTOWacc = msg.gpsTOWacc;
    out.gloTOWacc = msg.gloTOWacc;
    out.bdsTOWacc = msg.bdsTOWacc;
    out.qzssTOWacc = msg.qzssTOWacc;
    out.numSV = msg.numSV;
    out.flags = msg.flags;

    for(uint8_t svIndex=0; svIndex<msg.numSV; svIndex++)
    {
        ublox::Satellite sat;
        ublox::RXM_MEASX_t::SV_INFO_t msgsat = msg.sv[svIndex];
        sat.gnssID = msgsat.gnssID;
        sat.svID = msgsat.svID;
        sat.carrierNoiseRatio = msgsat.cNo;
        sat.multipathIndex = msgsat.mpathIndic;
        sat.dopplerMeas = msgsat.dopplerMS;
        sat.dopperHz = msgsat.dopplerHZ;
        sat.wholeChips = msgsat.wholeChips;
        sat.fracChips = msgsat.fracChips;
        sat.codePhase = msgsat.codePhase;
        sat.intCodePhase = msgsat.intCodePhase;
        sat.pseuRangeRMSError = msgsat.pseuRangeRMSErr;
        out.satellites.push_back(sat);
    }

    sat_status_pub_.publish(out);
}
}