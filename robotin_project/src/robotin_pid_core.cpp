#include "pid/pid_core.h"

RobotinPID::RobotinPID()
{
}

RobotinPID::~RobotinPID()
{
}

void RobotinPID::publishMessage(ros::Publisher *pub_message)
{
  robotin_project::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void RobotinPID::messageCallback(const robotin_project::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void RobotinPID::configCallback(robotin_project::RobotinPIDConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}
