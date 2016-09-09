/*
 * StrategyAgent.h
 *
 *  Created on: Nov 23, 2015
 *      Author: fwmav
 */

#ifndef STRATEGYAGENT_H_
#define STRATEGYAGENT_H_

#include "Agent.h"
#include <queue>
#include <math.h>
#include "Event.h"
#include <iostream>

#define WP_NB 1 // number of waypoints

#define WP_1_X 450 // px
#define WP_1_Y 250 // px

#define WP_2_X 400 // px
#define WP_2_Y 130 // px

#define WP_3_X 250 // px
#define WP_3_Y 130 // px

#define WP_4_X 250 // px
#define WP_4_Y 250 // px

#define WP_5_X 100 // px
#define WP_5_Y 250 // px

namespace std
{

  class StrategyAgent: public Agent
  {
  public:
    StrategyAgent()
    {
      wp_idx_ = 0;
      wp_x_ = 0;
      wp_y_ = 0;
      type_ = AGENT_STRATEGY;

      fillWaypoints();
    }

     StrategyAgent(queue<Event>* fifo)
     {
       wp_idx_ = 0;
       wp_x_ = 0;
       wp_y_ = 0;
       type_ = AGENT_STRATEGY;

       fillWaypoints();

       //pointer to event queue
       fifo_ = fifo;
     }

     /**
      * Stepped from the scheduler
      * return 1 if all OK (WP ready(
      * return 0 if an error occures
      */
     int step()
     {
         // stepped when a new waypoint is requested
         //increment waypoint
         wp_idx_++;
         if (wp_idx_ >= WP_NB) {
           wp_idx_ = 0;
         }
         // update internal data
         wp_x_ = wp_x[wp_idx_];
         wp_y_ = wp_y[wp_idx_];

       // post event into event queue
       fifo_->push(Event(EVENT_NEW_WP));

       return 1;
     }

     int get_wp_x();
     int get_wp_y();

   private:
     // waypoints list
     int wp_x[WP_NB];
     int wp_y[WP_NB];

     int wp_idx_;
     int wp_x_;
     int wp_y_;

     int vehicle_pos_x_;
     int vehicle_pos_y_;

     //pointer to event queue
     queue<Event>* fifo_;

     void fillWaypoints(){
#if WP_NB > 0
       wp_x[0] = WP_1_X;
       wp_y[0] = WP_1_Y;
#endif
#if WP_NB > 1
       wp_x[1] = WP_2_X;
       wp_y[1] = WP_2_Y;
#endif
#if WP_NB > 2
       wp_x[2] = WP_3_X;
       wp_y[2] = WP_3_Y;
#endif
#if WP_NB > 3
       wp_x[3] = WP_4_X;
       wp_y[3] = WP_4_Y;
#endif
#if WP_NB > 4
       wp_x[4] = WP_5_X;
       wp_y[4] = WP_5_Y;
#endif
#if WP_NB > 5
       wp_x[5] = WP_6_X;
       wp_y[5] = WP_6_Y;
#endif
// and so on

       // first waypoint
       wp_x_ = wp_x[0];
       wp_y_ = wp_y[0];
     }
  };

} /* namespace std */

#endif /* STRATEGYAGENT_H_ */
