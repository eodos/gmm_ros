#include "ros/ros.h"
#include "gmm/Odometry.h"
#include "gmm/Commands.h"

#include "aplink.hpp"

#include "BufferedAsyncSerial.h"
#include <boost/chrono/chrono.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

ros::Publisher pub;

void cmdCallback(gmm::Commands cmd)
{
    ROS_INFO("Forward velocity: [%f], Omega: [%f]", cmd.velocity, cmd.omega);
    ap::setCmd(cmd.velocity, cmd.omega);
}

void publishOdom(gmm::Odometry odom)
{
    ROS_INFO("AP packet received");
    pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gmm_aplink");
    ros::NodeHandle n;

    ros::Rate r(10000);
    pub = n.advertise<gmm::Odometry>("gmm/odom", 1);
    ros::Subscriber sub = n.subscribe("gmm/cmd_vel", 10, cmdCallback);

    ap::init();

    while(ros::ok())
    {
        ap::rxproc();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

namespace ap
{
    /// Serial device profile
    namespace dev
    {
        #define                         GULP    2UL
        static const char*              Name    = "/dev/ttyUSB0";
        static const size_t             Baud    = 115200UL;
        static const char*              Header  = "GMM-AP";
        static BufferedAsyncSerial      Port
        (
            Name,
            Baud,
            boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
            boost::asio::serial_port_base::character_size(8),
            boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),
            boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)
        );
    }

    /// Support variables (in lieu of a singleton object)
    static ret_t            ap_ret;
    static data_t           ap_data_copy;
    static cmd_t            ap_cmd;

    static void parseOdom()
    {
        gmm::Odometry odom;

        odom.header.stamp = ros::Time::now();

        odom.ap_time    = ap_ret.Data.val.time_stamp;
        odom.wheel_R    = ap_ret.Data.val.wheel_R;
        odom.wheel_L    = ap_ret.Data.val.wheel_L;
        odom.gyro.x     = *(ap_ret.Data.val.gyro);
        odom.gyro.y     = *(ap_ret.Data.val.gyro+1);
        odom.gyro.z     = *(ap_ret.Data.val.gyro+2);
        odom.acc1.x     = *(ap_ret.Data.val.acc1);
        odom.acc1.y     = *(ap_ret.Data.val.acc1+1);
        odom.acc1.z     = *(ap_ret.Data.val.acc1+2);
        odom.acc2.x     = *(ap_ret.Data.val.acc2);
        odom.acc2.y     = *(ap_ret.Data.val.acc2+1);
        odom.acc2.z     = *(ap_ret.Data.val.acc2+2);
        odom.magn.x     = *(ap_ret.Data.val.magn);
        odom.magn.y     = *(ap_ret.Data.val.magn+1);
        odom.magn.z     = *(ap_ret.Data.val.magn+2);
        odom.velocity   = ap_ret.Data.val.velocity;
        odom.omega      = ap_ret.Data.val.omega;

        // Publish message
        publishOdom(odom);
    }

    void init()
    {
        // Initialize communication
        ap_ret.header_len    = strlen(dev::Header)-1UL;
        ap_ret.header_idx    = 0UL;
        ap_ret.header        = (uint8_t*)(dev::Header);
        ap_ret.data_idx      = 0UL;
        ap_ret.data_len      = AP_DATALEN;
        ap_ret.checksum      = 0U;

        ROS_INFO("AutoPilot communication started");
    }

    static uint8_t calculateChecksum()
    {
        uint8_t retVal = 0U;;
        for( size_t idx = 0; idx < AP_CMDLEN; ++idx )
            retVal+=ap_cmd.Data.bytes[idx];

        return (0xFF&retVal);
    }

    static void txproc()
    {
        /// Write header
        dev::Port.write((const char*)(dev::Header),strlen(dev::Header)-1UL);

        /// Write command
        dev::Port.write((const char*)(ap_cmd.Data.bytes),AP_CMDLEN);

        /// Write checksum
        uint8_t cs = calculateChecksum();
        dev::Port.write((const char*)(&cs),1UL);
    }

    void rxproc()
    {
        char    tmpbuf[GULP];
        size_t  nBytes;

        /// Read some bytes from the device buffer
        nBytes    = dev::Port.read(tmpbuf,GULP);

        for( size_t idx = 0; idx < nBytes; ++idx )
        {
            /// Get next byte
            uint8_t recvdByte = static_cast<uint8_t>(tmpbuf[idx]);

            /// Get packet header
            if( ap_ret.header_idx < ap_ret.header_len )
            {
                if(ap_ret.header[ap_ret.header_idx]==recvdByte)
                    ++ap_ret.header_idx;
                else
                    ap_ret.header_idx = 0UL;
            }
            else
            {
                /// Get motor commands
                if(ap_ret.data_idx < ap_ret.data_len)
                {
                    ap_ret.checksum += recvdByte;
                    ap_ret.Data.bytes[ap_ret.data_idx++] = recvdByte;
                }
                else
                {
                    //sys::message("Data received.");
                    ap_ret.data_idx      =
                    ap_ret.header_idx = 0UL;

                    /// Validate + Generate working copy
                    if( ap_ret.checksum == recvdByte )
                    {
                        parseOdom();
                    }

                    ap_ret.checksum = 0U;
                }
            }
        }
    }

    void setCmd(const float& velocity, const float& omega)
    {
        ap_cmd.Data.val[0] = velocity;
        ap_cmd.Data.val[1] = omega;

        txproc();
    }
}