#ifndef _ROS_SERVICE_StartCapability_h
#define _ROS_SERVICE_StartCapability_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace capabilities
{

static const char STARTCAPABILITY[] = "capabilities/StartCapability";

  class StartCapabilityRequest : public ros::Msg
  {
    public:
      typedef const char* _capability_type;
      _capability_type capability;
      typedef const char* _preferred_provider_type;
      _preferred_provider_type preferred_provider;

    StartCapabilityRequest():
      capability(""),
      preferred_provider("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_capability = strlen(this->capability);
      varToArr(outbuffer + offset, length_capability);
      offset += 4;
      memcpy(outbuffer + offset, this->capability, length_capability);
      offset += length_capability;
      uint32_t length_preferred_provider = strlen(this->preferred_provider);
      varToArr(outbuffer + offset, length_preferred_provider);
      offset += 4;
      memcpy(outbuffer + offset, this->preferred_provider, length_preferred_provider);
      offset += length_preferred_provider;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_capability;
      arrToVar(length_capability, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_capability; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_capability-1]=0;
      this->capability = (char *)(inbuffer + offset-1);
      offset += length_capability;
      uint32_t length_preferred_provider;
      arrToVar(length_preferred_provider, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_preferred_provider; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_preferred_provider-1]=0;
      this->preferred_provider = (char *)(inbuffer + offset-1);
      offset += length_preferred_provider;
     return offset;
    }

    virtual const char * getType() override { return STARTCAPABILITY; };
    virtual const char * getMD5() override { return "2cc14acc01fae719c849d4ea89820c67"; };

  };

  class StartCapabilityResponse : public ros::Msg
  {
    public:
      typedef uint8_t _result_type;
      _result_type result;
      enum { RESULT_SUCCESS = 0 };
      enum { RESULT_CURRENTLY_STARTING = 1  };
      enum { RESULT_CURRENTLY_RUNNING = 2  };
      enum { RESULT_CURRENTLY_STOPPING = 3  };

    StartCapabilityResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->result >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->result =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return STARTCAPABILITY; };
    virtual const char * getMD5() override { return "987bf9617f6d43621fed499e5edfc366"; };

  };

  class StartCapability {
    public:
    typedef StartCapabilityRequest Request;
    typedef StartCapabilityResponse Response;
  };

}
#endif
