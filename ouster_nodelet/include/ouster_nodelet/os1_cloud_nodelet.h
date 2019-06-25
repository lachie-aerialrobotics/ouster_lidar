#ifndef OUSTER_CLOUD_NODELET_H_
#define OUSTER_CLOUD_NODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace ouster_nodelet{

class OS1CloudNodelet:public nodelet::Nodelet
{
    public:
    virtual void onInit();
    int run();
    
    private:

    bool validTimestamp(const ros::Time& msg_time);

    ros::NodeHandle nh;
    std::string tf_prefix_;
    std::string sensor_frame_;
    std::string imu_frame_;
    std::string lidar_frame_;


};

}
#endif  