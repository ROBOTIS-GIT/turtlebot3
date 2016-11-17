#ifndef _ROS_nav_msgs_GridCells_h
#define _ROS_nav_msgs_GridCells_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace nav_msgs
{

  class GridCells : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float cell_width;
      float cell_height;
      uint32_t cells_length;
      geometry_msgs::Point st_cells;
      geometry_msgs::Point * cells;

    GridCells():
      header(),
      cell_width(0),
      cell_height(0),
      cells_length(0), cells(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_cell_width;
      u_cell_width.real = this->cell_width;
      *(outbuffer + offset + 0) = (u_cell_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_width);
      union {
        float real;
        uint32_t base;
      } u_cell_height;
      u_cell_height.real = this->cell_height;
      *(outbuffer + offset + 0) = (u_cell_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_height);
      *(outbuffer + offset + 0) = (this->cells_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cells_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cells_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cells_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cells_length);
      for( uint32_t i = 0; i < cells_length; i++){
      offset += this->cells[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_cell_width;
      u_cell_width.base = 0;
      u_cell_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_width = u_cell_width.real;
      offset += sizeof(this->cell_width);
      union {
        float real;
        uint32_t base;
      } u_cell_height;
      u_cell_height.base = 0;
      u_cell_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_height = u_cell_height.real;
      offset += sizeof(this->cell_height);
      uint32_t cells_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cells_length);
      if(cells_lengthT > cells_length)
        this->cells = (geometry_msgs::Point*)realloc(this->cells, cells_lengthT * sizeof(geometry_msgs::Point));
      cells_length = cells_lengthT;
      for( uint32_t i = 0; i < cells_length; i++){
      offset += this->st_cells.deserialize(inbuffer + offset);
        memcpy( &(this->cells[i]), &(this->st_cells), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    const char * getType(){ return "nav_msgs/GridCells"; };
    const char * getMD5(){ return "b9e4f5df6d28e272ebde00a3994830f5"; };

  };

}
#endif