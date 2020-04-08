#ifndef SR_ROBOTIN_PID_CORE_H
#define SR_ROBOTIN_PID_CORE_H

#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include <robotin_project/PID.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <robotin_project/RobotinPIDConfig.h>

class RobotinPID
{
public:
  RobotinPID();
  ~RobotinPID();
  void configCallback(robotin_project::RobotinPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const robotin_project::PID::ConstPtr &msg);

  double p_;
  double d_;
  double i_;

};
#endif