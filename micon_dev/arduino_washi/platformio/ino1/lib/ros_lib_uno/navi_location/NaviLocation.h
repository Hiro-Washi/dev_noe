#ifndef _ROS_SERVICE_NaviLocation_h
#define _ROS_SERVICE_NaviLocation_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace navi_location
{

static const char NAVILOCATION[] = "navi_location/NaviLocation";

  class NaviLocationRequest : public ros::Msg
  {
    public:
      typedef const char* _location_name_type;
      _location_name_type location_name;

    NaviLocationRequest():
      location_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_location_name = strlen(this->location_name);
      varToArr(outbuffer + offset, length_location_name);
      offset += 4;
      memcpy(outbuffer + offset, this->location_name, length_location_name);
      offset += length_location_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_location_name;
      arrToVar(length_location_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_location_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_location_name-1]=0;
      this->location_name = (char *)(inbuffer + offset-1);
      offset += length_location_name;
     return offset;
    }

    virtual const char * getType() override { return NAVILOCATION; };
    virtual const char * getMD5() override { return "73faf335e0992a31df2e9630ffb73b0b"; };

  };

  class NaviLocationResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    NaviLocationResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return NAVILOCATION; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class NaviLocation {
    public:
    typedef NaviLocationRequest Request;
    typedef NaviLocationResponse Response;
  };

}
#endif
