/*
 * Event.h
 *
 *  Created on: Nov 20, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef EVENT_H_
#define EVENT_H_

namespace std
{

  class Event
  {
  private:
    int type;
    int value;
  public:
    Event(int type, int value){
      this->type = type;
      this->value = value;
    }
    int get_type();
    int get_value();
  };

  int Event::get_type() {
    return type;
  }

  int Event::get_value() {
    return value;
  }

} /* namespace std */

#endif /* EVENT_H_ */
