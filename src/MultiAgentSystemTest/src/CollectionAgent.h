/*
 * CollectionAgent.h
 *
 *  Created on: Nov 20, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef COLLECTIONAGENT_H_
#define COLLECTIONAGENT_H_

#include "Agent.h"
#include <queue>
#include "Event.h"

namespace std
{

  class CollectionAgent: public Agent
  {
  public:
    CollectionAgent(queue<Event>* fifo)
    {
      veh_pos_x_ = 0;
      veh_pos_y_ = 0;
      veh_heading_ = 0;
      veh_pos_x_filt_ = 0;
      veh_pos_y_filt_ = 0;
      veh_heading_filt_ = 0;

      //pointer to event queue
      fifo_ = fifo;
    }

    /**
     * Step function is triggerred by the scheduler
     * when appropriate event happends (such as "new
     * data available" )
     *
     * returns 1 if step was OK
     * returns 0 if error happened
     */
    int step()
    {
      // update coordinates, using simple low pass filter
      veh_pos_x_filt_ = alpha * veh_pos_x_ + (1 - alpha) * veh_pos_x_filt_;
      veh_pos_y_filt_ = alpha * veh_pos_y_ + (1 - alpha) * veh_pos_y_filt_;
      veh_heading_filt_ = alpha * veh_heading_
          + (1 - alpha) * veh_heading_filt_;

      // post event into queue
      // TODO
      return 1;
    }

    void set_pos(int x, int y, double phi);
    int get_pos_x();
    int get_pos_y();
    double get_heading();

  private:
    const static double alpha = 0.1;  // filter coefficient

    int veh_pos_x_;
    int veh_pos_y_;
    double veh_heading_;

    int veh_pos_x_filt_;
    int veh_pos_y_filt_;
    double veh_heading_filt_;

    //pointer to event queue
    queue<Event>* fifo_;
  };

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

#endif /* COLLECTIONAGENT_H_ */
