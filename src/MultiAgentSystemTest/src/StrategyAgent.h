/*
 * StrategyAgent.h
 *
 *  Created on: Nov 21, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef STRATEGYAGENT_H_
#define STRATEGYAGENT_H_

#include "Agent.h"
#include <queue>
#include "Event.h"

#define WP_NB 2 // number of waypoints
#define WP_1_X 100 // px
#define WP_1_Y 250 // px
#define WP_2_X 500 // px
#define WP_2_Y 250 // px

namespace std
{

  class StrategyAgent: public Agent
  {
  public:
    StrategyAgent(queue<Event>* fifo)
    {
      wp_idx_ = 0;
      wp_x_ = 0;
      wp_y_ = 0;

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
      // TODO

      return 1;
    }

    int get_wp_x();
    int get_wp_y();

  private:
    // waypoints list
    const static int wp_x[WP_NB] = { WP_1_X, WP_2_X };
    const static int wp_y[WP_NB] = { WP_1_Y, WP_2_Y };

    int wp_idx_;
    int wp_x_;
    int wp_y_;

    //pointer to event queue
    queue<Event>* fifo_;
  };

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

#endif /* STRATEGYAGENT_H_ */
