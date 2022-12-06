#ifndef _ROS_SERVICE_NaviCoord_h
#define _ROS_SERVICE_NaviCoord_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace happymimi_navigation
{

static const char NAVICOORD[] = "happymimi_navigation/NaviCoord";

  class NaviCoordRequest : public ros::Msg
  {
    public:
      uint32_t loc_coord_length;
      typedef float _loc_coord_type;
      _loc_coord_type st_loc_coord;
      _loc_coord_type * loc_coord;

    NaviCoordRequest():
      loc_coord_length(0), st_loc_coord(), loc_coord(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->loc_coord_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->loc_coord_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->loc_coord_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->loc_coord_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->loc_coord_length);
      for( uint32_t i = 0; i < loc_coord_length; i++){
      union {
        float real;
        uint32_t base;
      } u_loc_coordi;
      u_loc_coordi.real = this->loc_coord[i];
      *(outbuffer + offset + 0) = (u_loc_coordi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_loc_coordi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_loc_coordi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_loc_coordi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->loc_coord[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t loc_coord_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      loc_coord_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      loc_coord_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      loc_coord_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->loc_coord_length);
      if(loc_coord_lengthT > loc_coord_length)
        this->loc_coord = (float*)realloc(this->loc_coord, loc_coord_lengthT * sizeof(float));
      loc_coord_length = loc_coord_lengthT;
      for( uint32_t i = 0; i < loc_coord_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_loc_coord;
      u_st_loc_coord.base = 0;
      u_st_loc_coord.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_loc_coord.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_loc_coord.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_loc_coord.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_loc_coord = u_st_loc_coord.real;
      offset += sizeof(this->st_loc_coord);
        memcpy( &(this->loc_coord[i]), &(this->st_loc_coord), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return NAVICOORD; };
    virtual const char * getMD5() override { return "06aaf524d004b42fe9541758311ce6dc"; };

  };

  class NaviCoordResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    NaviCoordResponse():
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

    virtual const char * getType() override { return NAVICOORD; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class NaviCoord {
    public:
    typedef NaviCoordRequest Request;
    typedef NaviCoordResponse Response;
  };

}
#endif
