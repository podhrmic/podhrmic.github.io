/*
 * ControlAgent.h
 *
 *  Created on: Nov 23, 2015
 *      Author: fwmav
 */

#ifndef CONTROLAGENT_H_
#define CONTROLAGENT_H_

#include "Agent.h"
#include <queue>
#include <vector>
#include "Event.h"
#include <math.h>
#include <stdlib.h>     /* abs */
#include <iostream>

// for the status
#include "serverthread.h"

#define PI 3.14159265358979323846

#define RULE_IDLE 0
#define RULE_TURN_LEFT 1
#define RULE_TURN_RIGHT 2
#define RULE_GO_FORWARD 3
#define RULE_AVOID_OBSTACLE 4

// normal operation
#define CONTROL_DEG_THRESHOLD 20 //deg
#define CONTROL_DELTA_ACTIVE_LEFT 12 // 14 for recovery
#define CONTROL_DELTA_ACTIVE_RIGHT 14
#define CONTROL_DELTA_NEUTRAL 0
#define CONTROL_OMEGA_NEUTRAL 12
#define CONTROL_OMEGA_ACTIVE_LEFT 25 // 30 for recovery
#define CONTROL_OMEGA_ACTIVE_RIGHT 30

// recovery values
#define CONTROL_DELTA_ACTIVE_LEFT_RECOVERY 14
#define CONTROL_DELTA_ACTIVE_RIGHT_RECOVERY 14 // the same
#define CONTROL_DELTA_NEUTRAL_RECOVERY 0 // the same
#define CONTROL_OMEGA_NEUTRAL_RECOVERY 12 // the same
#define CONTROL_OMEGA_ACTIVE_LEFT_RECOVERY 30
#define CONTROL_OMEGA_ACTIVE_RIGHT_RECOVERY 30 // the same


#define DISTANCE_THRESHOLD 70.0 // px

// Obstacle coordinates
#define USE_OBSTACLE 0
#define OBSTACLE_SIZE 2 // line
#define OBSTACLE_X_1 320
#define OBSTACLE_X_2 320
#define OBSTACLE_Y_1 200
#define OBSTACLE_Y_2 300

enum ControlAgentStatus {
  UNINIT,
  INIT,
  TEST_START,
  TEST_RUNNING,
  TEST_END,
  RUN
};

namespace std
{

  class ControlAgent: public Agent
  {
      public:
    ControlAgent(){
      agentInit();
    }

    ControlAgent(queue<Event>* fifo){
      agentInit();

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

      switch(agent_status){
        case UNINIT:
          // do nothing unless the server is ready
          if ((server_status != SEND_INIT) && (cur_time > 10.0)) {
            // IDLE is default
            // if not IDLE then we are probably initialized
            agent_status = INIT;
            rule_ = ruleIdle(); // just idle until initialized
          }
          break;
        case INIT:
          // lets start with the measurement
          // record the initial conditions
          // and start turning
          if ((cur_time != 0.0) && (orientation_ != 0.0)) {
            d_time = cur_time;
            d_orientation = orientation_;
            agent_status = TEST_START;
            // make right turn
            rule_ = ruleTurnRight();
          }
          break;
        case TEST_START:
          // wait for a few seconds
          if ((cur_time - d_time) > 20.0){
            // 10 seconds passed, lets move to the running phase
            agent_status = TEST_RUNNING;
            rule_ = ruleTurnRight();
          }
          break;
        case TEST_RUNNING:
          // check with 5 degrees tolerance
          if ( fabs(orientation_ - d_orientation) < 5.0 ) {
            // we turned around
            // d_time is how long the turn took
            final_time = cur_time - d_time;
            // test ended, just idle for now
            agent_status = TEST_END;
            rule_ = ruleIdle();
          }
          break;
        case TEST_END:
          // check what the time of turn was, adjust if needed
          // lets do cutoff for right turn at 1:30 which is 90s
          if (final_time > 90.0) {
            damaged = true;
          }
          else {
            damaged = false;
          }
          agent_status = RUN;
          rule_ = ruleIdle();
          break;
        case RUN:
          // normal run
          // arrived at the destination?
          wp_reached_ = wpReached();
          if(wp_reached_) {
            // yes -> post an event into event queue and return
            fifo_->push(Event(EVENT_WP_REACHED));
            rule_ = ruleIdle();
            return 1;
          }

          // obstacle?
          close_to_obstacle_ = obstacleAhead();

          if (close_to_obstacle_ && USE_OBSTACLE){
            rule_ = ruleAvoidObstacle();
          }
          else {
            // calculate heading, trigger the correct rule
            rule_ = controlLoop();
          }
          break;
        default:
          // do nothing
          break;
      }

      // post the correct event into queue
      // set the delta/omega values
      fifo_->push(Event(EVENT_RULE_THAT_FIRED));
      fifo_->push(Event(EVENT_CONTROL_UPDATE));

      return 1;
    }

    // This function converts decimal degrees to radians
    static double deg2rad(double deg) {
      return (deg * PI / 180);
    }

    //  This function converts radians to decimal degrees
    static double rad2deg(double rad) {
      return (rad * 180 / PI);
    }

    void agentInit(){
      // position
      pos_x_ = 0;
      pos_y_ = 0;
      orientation_ = 0;
      dist_ = 0.0;

      // desired location
      wp_x_ = 0;
      wp_y_ = 0;

      // desired orientation
      desired_course_ = 0;
      course_err_ = 0;

      rule_ = 0;

      delta_left = 0;
      delta_right = 0;
      omega_left = 0;
      omega_right = 0;

      type_ = AGENT_CONTROL;

      // obstacle point 1
      obstacle_x_.push_back(OBSTACLE_X_1);
      obstacle_y_.push_back(OBSTACLE_Y_1);

      // obstacle point 2
      obstacle_x_.push_back(OBSTACLE_X_2);
      obstacle_y_.push_back(OBSTACLE_Y_2);

      close_to_obstacle_ = false;

      // server and agent status
      server_status = IDLE; // that is the default
      agent_status = UNINIT;

      // diagnostics data
      d_orientation = 0.0;
      d_time = 0.0;
      cur_time = 0.0;
      final_time = 0.0;
      damaged = false;
    }

    void set_pos(int x, int y, double phi);
    void set_wp(int x, int y);
    int getRule();

    float getDeltaLeft();
    float getDeltaRight();
    float getOmegaLeft();
    float getOmegaRight();
    const char* getRuleString();
    double getDesiredCourse();
    double getCourseError();
    vector<int> getObstacleX();
    vector<int> getObstacleY();
    int get_wp_x();
    int get_wp_y();
    double getDistance();
    bool has_reached_wp();

    // the status
    enum ServerStatus server_status;
    enum ControlAgentStatus agent_status;

    // diagnostics data
    float d_orientation;
    float d_time;
    float cur_time;
    float final_time;
    bool damaged;

  private:
    // position
    int pos_x_;
    int pos_y_;
    double orientation_;

    // obstacle
    vector<int> obstacle_x_;
    vector<int> obstacle_y_;
    double dist_;

    // desired location
    int wp_x_;
    int wp_y_;

    // desired orientation
    double desired_course_;

    // course error
    double course_err_;

    // rule that fired
    int rule_;

    // check for obstacle
    bool close_to_obstacle_;

    static const char* rule_idle;
    static const char* rule_turn_left;
    static const char* rule_turn_right;
    static const char* rule_go_forward;
    static const char* rule_avoid_obstacle;
    static const char* rule_string;

    float delta_left;
    float delta_right;
    float omega_left;
    float omega_right;

    bool wp_reached_;

    //pointer to event queue
    queue<Event>* fifo_;

    int ruleTurnLeft();
    int ruleTurnRight();
    int ruleGoForward();
    int ruleIdle();
    int ruleAvoidObstacle();
    void findOrientation();
    int controlLoop();
    bool wpReached();
    bool obstacleAhead();
  };

} /* namespace std */

#endif /* CONTROLAGENT_H_ */
