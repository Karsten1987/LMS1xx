#include <csignal>
#include <cstdio>
#include <LMS1xx/LMS1xx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/algorithm/string.hpp>

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  scanCfg scan_config;
  scanOutputRange output_config;

  // parameters
  std::string host;

  const std::string& node_name = "lms1xx_configuration";
  ros::init(argc, argv, node_name);
  ros::NodeHandle n("~");

  n.param<std::string>("host", host, "192.168.1.2");
  int time = 10;
  n.param<int>("time_out", time, time);

  ros::Duration time_out(time);
  ros::Time starting_time = ros::Time::now();

  // reading current SICK configuration
  while(!laser.isConnected()) {
    ros::Duration d(ros::Time::now()-starting_time);
    if (d>time_out)
    {
      ROS_ERROR_STREAM_NAMED(node_name, "Time out while trying to connect to laser " << host);
      exit(-1);
    }

    laser.connect(host);
    if(laser.isConnected())
    {
      laser.scanContinous(0);
      laser.stopMeas();
      laser.login();
      scan_config = laser.getScanCfg();
    }
    ROS_INFO_STREAM_NAMED(node_name, "Waiting for laser to connect!");
  }

  // read new configuration from param server
  n.param<int>("scaning_frequency",  scan_config.scaningFrequency, scan_config.scaningFrequency);
  n.param<int>("angle_resolution",   scan_config.angleResolution,  scan_config.angleResolution);
  n.param<int>("start_angle",        scan_config.startAngle,       scan_config.startAngle);
  n.param<int>("stop_angle",         scan_config.stopAngle,        scan_config.stopAngle);
  bool make_persistent = false;
  n.param<bool>("make_persistent",   make_persistent,              make_persistent);
  std::string ip_address;
  n.param<std::string>("ip_address", ip_address,                   ip_address);

  if (ip_address != host)
  {
    ROS_WARN_STREAM_NAMED(node_name, "You specified a different IP address."
        << "Going to reset the IP address of the laser to " << ip_address << ". "
        << "After that, you won't be able to connect to your laser with this address: " << host);

    std::vector<std::string> ip_octet_strings;
    boost::split(ip_octet_strings, ip_address, boost::is_any_of("."));
    if (ip_octet_strings.size() !=4)
    {
      ROS_ERROR_STREAM_NAMED(node_name, "IP address is malformed! Should be in form of <xxx>.<xxx>.<xxx>.<xxx>");
      exit(-1);
    }

    ipCfg ip_config;
    ip_config.oct_0 = atoi(ip_octet_strings[0].c_str());
    ip_config.oct_1 = atoi(ip_octet_strings[1].c_str());
    ip_config.oct_2 = atoi(ip_octet_strings[2].c_str());
    ip_config.oct_3 = atoi(ip_octet_strings[3].c_str());

    ROS_DEBUG_STREAM("Gonna send this octets " << ip_config.oct_0 << " " << ip_config.oct_1 << " " << ip_config.oct_2 << " " << ip_config.oct_3);
    laser.setIP(ip_config);

    laser.reboot();
    ROS_WARN_STREAM_NAMED(node_name, "New IP address set! Rebooting device now. Please reconnect with the new IP: " << ip_address);
    return 0;
  }

  output_config.angleResolution = scan_config.angleResolution;
  output_config.startAngle      = scan_config.startAngle;
  output_config.stopAngle       = scan_config.stopAngle;

  laser.setScanCfg(scan_config);
  laser.setOutputRange(output_config);
  if (make_persistent)
  {
    laser.saveConfig(); // make permanently
  }

  scan_config = laser.getScanCfg();
  output_config = laser.getScanOutputRange();

  ROS_INFO("New laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
                scan_config.scaningFrequency, scan_config.angleResolution, scan_config.startAngle, scan_config.stopAngle);
  ROS_INFO("New laser output range:angleResolution %d, startAngle %d, stopAngle %d",
                output_config.angleResolution, output_config.startAngle, output_config.stopAngle);

  ROS_INFO_STREAM_NAMED(node_name, "Shutting down laser " << host);
  laser.disconnect();

  return 0;
}
