/*
 * Triangle.h
 *
 *  Created on: Nov 13, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include "Shape.h"

namespace std
{

  class Triangle: public Shape
  {
  public:
    int getArea()
    {
       return (width * height)/2;
    }
  };

} /* namespace std */

#endif /* TRIANGLE_H_ */
