/*
 * ControlAgent.h
 *
 *  Created on: Nov 21, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef CONTROLAGENT_H_
#define CONTROLAGENT_H_

#include "Agent.h"
#include <queue>
#include "Event.h"

namespace std
{

  class ControlAgent: public Agent
  {
  public:
    ControlAgent(int wp_x, int wp_y, queue<Event>* fifo){
      // position
      pos_x_ = 0;
      pos_y_ = 0;
      orientation_ = 0;

      // desired location
      wp_x_ = wp_x;
      wp_y_ = wp_y;

      // desired orientation
      desired_phi_ = 0;

      //pointer to event queue
      fifo_ = fifo;
    }
    /**
     * During the step, these things should happen:
     * 1) check if we arrived to new destination
     * 2) check if we are clear from obstacles
     * 3) determine which rule to use
     * 4) set delta/omega values
     *
     * return 1 if OK
     * return 0 if error occured
     */
    int step(){
      // arrived at the destination?
      // yes -> post an event into event queue and return

      // obstacle?
      // yes -> turn right
      // else
      // calculate heading, trigger the correct rule
      // post the correct event into queue

      // set the delta/omega values

      return 1;
    }

  private:
    // position
    int pos_x_;
    int pos_y_;
    double orientation_;

    // desired location
    int wp_x_;
    int wp_y_;

    // desired orientation
    double desired_phi_;

    //pointer to event queue
    queue<Event>* fifo_;

  };

} /* namespace std */

#endif /* CONTROLAGENT_H_ */
