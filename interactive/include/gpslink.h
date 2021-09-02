#include <iostream>

#include <me120_msgs/gpsPacket.h>

class GPSLinkdata{
public:
    std_msgs::Header _header_msg;
    float UTC;        // hhmmss.ss  UTC time
    double _latitude;  // Format: dd.dddd; -90 ~ 90 (positive to north, negative to
                        // south)
    double _longitude;  // Format: ddd.dddd;-180 ~ 180 (positive to east, negative
                        // to west)
    double _altitude;
    double _heading;     // -180 ~ 180-->rad, east x, north y .
    double _pitch;       // -90 ~ 90
    double _roll;        // -180 ~ 180
    double _Ve;          // velocity in east direction (m/s)
    double _Vn;          // velocity in north direction (m/s)
    float _roti;        // rate of turn (degree/min) 这个怎么计算。
    uint8_t _status;    // GPS quality

    GPSLinkdata();
    virtual ~GPSLinkdata();
    void setData(me120_msgs::gpsPacket PacketMsg);
};