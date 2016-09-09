/*
 * StrategyAgent.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: fwmav
 */

#include "StrategyAgent.h"

namespace std
{
  /**
    * returns x position of new waypoint
    */
   int StrategyAgent::get_wp_x()
   {
     return wp_x_;
   }

   /**
    * returns y position of new waypoint
    */
   int StrategyAgent::get_wp_y()
   {
     return wp_y_;
   }


} /* namespace std */
