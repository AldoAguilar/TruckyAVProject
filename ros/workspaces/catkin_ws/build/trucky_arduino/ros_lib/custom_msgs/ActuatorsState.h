#ifndef _ROS_custom_msgs_ActuatorsState_h
#define _ROS_custom_msgs_ActuatorsState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

  class ActuatorsState : public ros::Msg
  {
    public:
      typedef int64_t _servo_pwm_high_time_type;
      _servo_pwm_high_time_type servo_pwm_high_time;
      typedef int64_t _motor_pwm_high_time_type;
      _motor_pwm_high_time_type motor_pwm_high_time;
      typedef const char* _output_mode_type;
      _output_mode_type output_mode;

    ActuatorsState():
      servo_pwm_high_time(0),
      motor_pwm_high_time(0),
      output_mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_servo_pwm_high_time;
      u_servo_pwm_high_time.real = this->servo_pwm_high_time;
      *(outbuffer + offset + 0) = (u_servo_pwm_high_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_pwm_high_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_pwm_high_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_pwm_high_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_servo_pwm_high_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_servo_pwm_high_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_servo_pwm_high_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_servo_pwm_high_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->servo_pwm_high_time);
      union {
        int64_t real;
        uint64_t base;
      } u_motor_pwm_high_time;
      u_motor_pwm_high_time.real = this->motor_pwm_high_time;
      *(outbuffer + offset + 0) = (u_motor_pwm_high_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_pwm_high_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_pwm_high_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_pwm_high_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_motor_pwm_high_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_motor_pwm_high_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_motor_pwm_high_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_motor_pwm_high_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->motor_pwm_high_time);
      uint32_t length_output_mode = strlen(this->output_mode);
      varToArr(outbuffer + offset, length_output_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->output_mode, length_output_mode);
      offset += length_output_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_servo_pwm_high_time;
      u_servo_pwm_high_time.base = 0;
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_servo_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->servo_pwm_high_time = u_servo_pwm_high_time.real;
      offset += sizeof(this->servo_pwm_high_time);
      union {
        int64_t real;
        uint64_t base;
      } u_motor_pwm_high_time;
      u_motor_pwm_high_time.base = 0;
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_motor_pwm_high_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->motor_pwm_high_time = u_motor_pwm_high_time.real;
      offset += sizeof(this->motor_pwm_high_time);
      uint32_t length_output_mode;
      arrToVar(length_output_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_output_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_output_mode-1]=0;
      this->output_mode = (char *)(inbuffer + offset-1);
      offset += length_output_mode;
     return offset;
    }

    const char * getType(){ return "custom_msgs/ActuatorsState"; };
    const char * getMD5(){ return "4b99dd1ce38e6a3931dbead99360d717"; };

  };

}
#endif
