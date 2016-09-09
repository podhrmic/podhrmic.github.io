/*
 * Event.h
 *
 *  Created on: Nov 23, 2015
 *      Author: fwmav
 */

#ifndef EVENT_H_
#define EVENT_H_

#define EVENT_NEW_DATA 0
#define EVENT_NEW_FILTERED_DATA 1
#define EVENT_WP_REACHED 2
#define EVENT_NEW_WP 3
#define EVENT_RULE_THAT_FIRED 4
#define EVENT_CONTROL_UPDATE 5

namespace std
{

  class Event
  {
  private:
    int type_;
    int value_;
  public:
    Event(){
          type_ = 0;
          value_ = 0;
        }
    Event(int type){
          type_ = type;
          value_ = 0;
        }
    Event(int type, int value){
      type_ = type;
      value_ = value;
    }
    int get_type();
    int get_value();
  };

} /* namespace std */

#endif /* EVENT_H_ */
