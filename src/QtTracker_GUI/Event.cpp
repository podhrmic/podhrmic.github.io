/*
 * Event.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: fwmav
 */

#include "Event.h"

namespace std
{
  int Event::get_type() {
    return type_;
  }

  int Event::get_value() {
    return value_;
  }

} /* namespace std */
