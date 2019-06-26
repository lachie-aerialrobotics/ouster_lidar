#ifndef OUSTER_ASSEMBLER_NODELET_H_
#define OUSTER_ASSEMBLER_NODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Dumping 
#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <ouster/os1_packet.h>
#include <ouster/os1_util.h>
#include <ouster_ros/OS1ConfigSrv.h>
#include <ouster_ros/PacketMsg.h>
#include <ouster_ros/os1_ros.h>


using ns = std::chrono::nanoseconds;
using PacketMsg = ouster_ros::PacketMsg;
using OS1ConfigSrv = ouster_ros::OS1ConfigSrv;
namespace OS1 = ouster::OS1;

namespace ouster_nodelet
{

class OS1AssemblerNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit();

private:
    int run();

    bool validTimestamp(const ros::Time &msg_time);
    int connection_loop(ros::NodeHandle &nh, OS1::client &cli);
    void write_metadata(const std::string &meta_file, const std::string &metadata);
    std::string read_metadata(const std::string &meta_file);
    void populate_metadata_defaults(OS1::sensor_info &info,
                                    const std::string &specified_lidar_mode);
   
   // Refrence to private node handle
   ros::NodeHandle nh;
   bool debug_;                                 
};

} // namespace ouster_nodelet
#endif