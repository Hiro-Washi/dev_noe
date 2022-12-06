#ifndef _ROS_roscpp_ex_adder_h
#define _ROS_roscpp_ex_adder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roscpp_ex
{

  class adder : public ros::Msg
  {
    public:
      typedef uint16_t _a_type;
      _a_type a;
      typedef uint16_t _b_type;
      _b_type b;

    adder():
      a(0),
      b(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->a >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->a >> (8 * 1)) & 0xFF;
      offset += sizeof(this->a);
      *(outbuffer + offset + 0) = (this->b >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->b >> (8 * 1)) & 0xFF;
      offset += sizeof(this->b);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->a =  ((uint16_t) (*(inbuffer + offset)));
      this->a |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->a);
      this->b =  ((uint16_t) (*(inbuffer + offset)));
      this->b |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->b);
     return offset;
    }

    virtual const char * getType() override { return "roscpp_ex/adder"; };
    virtual const char * getMD5() override { return "e689bd5e4bb85cf611fb121d2a0d923b"; };

  };

}
#endif
