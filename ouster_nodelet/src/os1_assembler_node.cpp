/**
 * 
 *  \author     Fabio Ruetz <fabio.ruetz@asl-ethz.ch>
 * 
 *  Node implementation of a nodelet. This allows it to be launched as node from the launch file. 
 *  This node is based on https://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html
 * 
 * 
 */

#include <ros/ros.h>
#include <nodelet/loader.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "os1_assembler_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "ouster_nodelet/os1_assembler_nodelet", remap, nargv);
  // ros::spin();

  return 0;

}