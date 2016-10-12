/*
 * Shape.h
 *
 *  Created on: Nov 13, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef SHAPE_H_
#define SHAPE_H_

namespace std
{

  class Shape
  {
  public:
    // pure virtual function providing interface framework.
    virtual int getArea() = 0;
    void setWidth(int w)
    {
       width = w;
    }
    void setHeight(int h)
    {
       height = h;
    }
 protected:
    int width;
    int height;
  };

} /* namespace std */

#endif /* SHAPE_H_ */
