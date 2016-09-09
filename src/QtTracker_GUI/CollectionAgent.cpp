/*
 * CollectionAgent.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: fwmav
 */

#include "CollectionAgent.h"

namespace std
{

  /**
   * Set new raw position (from camera)
   */
  void CollectionAgent::set_pos(int x, int y, double phi)
  {
    veh_pos_x_ = x;
    veh_pos_y_ = y;
    veh_heading_ = phi;
  }

  /**
   * Return filtered position
   */
  int CollectionAgent::get_pos_x()
  {
    return veh_pos_x_filt_;
  }

  /**
   * Return filtered position
   */
  int CollectionAgent::get_pos_y()
  {
    return veh_pos_y_filt_;
  }

  /**
   * Return filtered position
   */
  double CollectionAgent::get_heading()
  {
    return veh_heading_filt_;
  }
} /* namespace std */
