#ifndef _ROS_robotin_project_TEL_h
#define _ROS_robotin_project_TEL_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotin_project
{

  class TEL : public ros::Msg
  {
    public:
      typedef float _target_velocity_type;
      _target_velocity_type target_velocity;
      typedef float _current_velocity_type;
      _current_velocity_type current_velocity;
      typedef float _demanded_velocity_type;
      _demanded_velocity_type demanded_velocity;
      typedef uint32_t _demanded_duty_type;
      _demanded_duty_type demanded_duty;

    TEL():
      target_velocity(0),
      current_velocity(0),
      demanded_velocity(0),
      demanded_duty(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_velocity;
      u_target_velocity.real = this->target_velocity;
      *(outbuffer + offset + 0) = (u_target_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_velocity);
      union {
        float real;
        uint32_t base;
      } u_current_velocity;
      u_current_velocity.real = this->current_velocity;
      *(outbuffer + offset + 0) = (u_current_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_velocity);
      union {
        float real;
        uint32_t base;
      } u_demanded_velocity;
      u_demanded_velocity.real = this->demanded_velocity;
      *(outbuffer + offset + 0) = (u_demanded_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_demanded_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_demanded_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_demanded_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->demanded_velocity);
      *(outbuffer + offset + 0) = (this->demanded_duty >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->demanded_duty >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->demanded_duty >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->demanded_duty >> (8 * 3)) & 0xFF;
      offset += sizeof(this->demanded_duty);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_velocity;
      u_target_velocity.base = 0;
      u_target_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_velocity = u_target_velocity.real;
      offset += sizeof(this->target_velocity);
      union {
        float real;
        uint32_t base;
      } u_current_velocity;
      u_current_velocity.base = 0;
      u_current_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_velocity = u_current_velocity.real;
      offset += sizeof(this->current_velocity);
      union {
        float real;
        uint32_t base;
      } u_demanded_velocity;
      u_demanded_velocity.base = 0;
      u_demanded_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_demanded_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_demanded_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_demanded_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->demanded_velocity = u_demanded_velocity.real;
      offset += sizeof(this->demanded_velocity);
      this->demanded_duty =  ((uint32_t) (*(inbuffer + offset)));
      this->demanded_duty |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->demanded_duty |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->demanded_duty |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->demanded_duty);
     return offset;
    }

    const char * getType(){ return "robotin_project/TEL"; };
    const char * getMD5(){ return "2e49a10223afa0461b37939f53fc5c0c"; };

  };

}
#endif
