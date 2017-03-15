/**
 * @file /turtlebot_parnorama/include/turtlebot_panorama/geometry.h
 *
 * @brief Simple geometry functions.
 *
 * @date Nov 12, 2012
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef TURTLEBOT_PANORAMA_GEOMETRY_H_
#define TURTLEBOT_PANORAMA_GEOMETRY_H_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cmath>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace turtlebot_panorama
{

/*****************************************************************************
 ** Interfaces
 *****************************************************************************/

template<typename T>
  T degrees_to_radians(const T &degrees)
  {
    static const double degs_to_rads = M_PI / 180.0;
    return degrees * degs_to_rads;
  }

template<typename T>
  T radians_to_degrees(const T &radians)
  {
    static const double rads_to_degs = 180.0 / M_PI;
    return radians * rads_to_degs;
  }

template<typename T>
  T wrap_angle(const T &angle)
  {
    float wrapped;
    if ((angle <= M_PI) && (angle >= -M_PI))
    {
      wrapped = angle;
    }
    else if (angle < 0.0)
    {
      wrapped = fmodf(angle - M_PI, 2.0 * M_PI) + M_PI;
    }
    else
    {
      wrapped = fmodf(angle + M_PI, 2.0 * M_PI) - M_PI;
    }
    return wrapped;
  }

} // namespace turtlebot_panorama

#endif /* TURTLEBOT_PANORAMA_GEOMETRY_H_ */
