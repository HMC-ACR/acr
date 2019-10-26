#ifndef _ROS_low_level_theta_dot_lr_h
#define _ROS_low_level_theta_dot_lr_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace low_level
{

  class theta_dot_lr : public ros::Msg
  {
    public:
      typedef float _theta_dot_left_type;
      _theta_dot_left_type theta_dot_left;
      typedef float _theta_dot_right_type;
      _theta_dot_right_type theta_dot_right;

    theta_dot_lr():
      theta_dot_left(0),
      theta_dot_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_theta_dot_left;
      u_theta_dot_left.real = this->theta_dot_left;
      *(outbuffer + offset + 0) = (u_theta_dot_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta_dot_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta_dot_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta_dot_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta_dot_left);
      union {
        float real;
        uint32_t base;
      } u_theta_dot_right;
      u_theta_dot_right.real = this->theta_dot_right;
      *(outbuffer + offset + 0) = (u_theta_dot_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta_dot_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta_dot_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta_dot_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta_dot_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_theta_dot_left;
      u_theta_dot_left.base = 0;
      u_theta_dot_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta_dot_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta_dot_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta_dot_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta_dot_left = u_theta_dot_left.real;
      offset += sizeof(this->theta_dot_left);
      union {
        float real;
        uint32_t base;
      } u_theta_dot_right;
      u_theta_dot_right.base = 0;
      u_theta_dot_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta_dot_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta_dot_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta_dot_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta_dot_right = u_theta_dot_right.real;
      offset += sizeof(this->theta_dot_right);
     return offset;
    }

    const char * getType(){ return "low_level/theta_dot_lr"; };
    const char * getMD5(){ return "bd5df01c85a05c89a29d456cc9a16601"; };

  };

}
#endif
