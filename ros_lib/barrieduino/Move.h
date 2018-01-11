#ifndef _ROS_barrieduino_Move_h
#define _ROS_barrieduino_Move_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace barrieduino
{

  class Move : public ros::Msg
  {
    public:
      typedef uint8_t _lane_type;
      _lane_type lane;
      typedef uint8_t _location_type;
      _location_type location;

    Move():
      lane(0),
      location(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->lane >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lane);
      *(outbuffer + offset + 0) = (this->location >> (8 * 0)) & 0xFF;
      offset += sizeof(this->location);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->lane =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->lane);
      this->location =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->location);
     return offset;
    }

    const char * getType(){ return "barrieduino/Move"; };
    const char * getMD5(){ return "073cd3e5df61e2bff607d7b2f3695734"; };

  };

}
#endif