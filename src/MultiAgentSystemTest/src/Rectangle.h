/*
 * Rectangle.h
 *
 *  Created on: Nov 13, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef RECTANGLE_H_
#define RECTANGLE_H_

#include "Shape.h"

namespace std
{

  class Rectangle: public Shape
  {
  public:
     int getArea()
     {
        return (width * height);
     }
  };

} /* namespace std */

#endif /* RECTANGLE_H_ */
