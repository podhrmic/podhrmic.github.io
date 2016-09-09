/*
 * ControlAgent.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: fwmav
 */

#include "ControlAgent.h"

namespace std
{
  const char* ControlAgent::rule_idle = "IDLE";
  const char* ControlAgent::rule_turn_left = "TURN LEFT";
  const char* ControlAgent::rule_turn_right = "TURN RIGHT";
  const char* ControlAgent::rule_go_forward = "GO FORWARD";
  const char* ControlAgent::rule_avoid_obstacle = "AVOID OBSTACLE";

  /**
   * Set new vehicle position and orientation
   */
  void ControlAgent::set_pos(int x, int y, double phi)
  {
    pos_x_ = x;
    pos_y_ = y;
    orientation_ = phi;
  }

  /**
   * Set new waypoint
   */
  void ControlAgent::set_wp(int x, int y)
  {
    wp_x_ = x;
    wp_y_ = y;
  }

  int ControlAgent::get_wp_x()
  {
    return wp_x_;
  }

  int ControlAgent::get_wp_y()
  {
    return wp_y_;
  }

  /**
   * Calculate desired orientation
   */
  void ControlAgent::findOrientation()
  {
    // calculate desired course
    static double first_term = 0;
    static double second_term = 0;
    static double cosphi_tmp = 0;
    static double root = 0;

    //Point u = Point(1, 0);
    int ux = 1;
    int uy = 0;

    int vx = 0;
    int vy = 0;
    if (!close_to_obstacle_){
      // normal run, use stored position
      vx = wp_x_ - pos_x_;
      vy = wp_y_ - pos_y_;
    }
    else{
      // avoiding obstacle
      if (pos_x_ > OBSTACLE_X_1) {
        // right side
        if (pos_y_ > 150){
          // far from the edge
          // go to above (350, 100)
          vx = 350 - pos_x_;
          vy = 100 - pos_y_;
        }
        else {
          // close to the edge
          // go left (150, 100)
          vx = 150 - pos_x_;
          vy = 100 - pos_y_;
        }
      }
    }

    first_term = ux * vx;
    second_term = uy * vy;
    cosphi_tmp = (first_term + second_term);
    root = (sqrt(ux * ux + uy * uy) * sqrt(vx * vx + vy * vy));
    cosphi_tmp = cosphi_tmp / root;
    desired_course_ = rad2deg(acos(cosphi_tmp));
    // check sign
    if (vy < 0) {
      desired_course_ = -1 * desired_course_;
    }
  }

  /**
   * Main control loop
   */
  int ControlAgent::controlLoop()
  {
    findOrientation();
    // course correction
    course_err_ = orientation_ - desired_course_;
    double cmpl_course_err = 360 - fabs(course_err_);

    if (cmpl_course_err < fabs(course_err_)) {
      course_err_ = -1 * course_err_;
    }

    // now we are just turning left/right
    //return ruleTurnLeft();

    if (course_err_ > (CONTROL_DEG_THRESHOLD+10)) {
      // >>> Turn left
      return ruleTurnLeft();
    } else {
      if (course_err_ < - (CONTROL_DEG_THRESHOLD-10)) {
        //  >>> turn right
        return ruleTurnRight();
      } else {
        //  >>>> go straight
        return ruleGoForward();
      }
    }
  }

  /**
   * Helper function
   * Have we arrived at the destination yet?
   */
  bool ControlAgent::wpReached()
  {
    // reached waypoint?
    dist_ = 0.0;
    dist_ = sqrt(pow((pos_x_ - wp_x_), 2.0) + pow((pos_y_ - wp_y_), 2.0));
    if (dist_ < DISTANCE_THRESHOLD) {
      return true;
    } else {
      return false;
    }
  }

  double ControlAgent::getDistance(){
    return dist_;
  }


  /**
   * Is there an obstacle to avoid?
   */
  bool ControlAgent::obstacleAhead()
  {
    // simulate a proximity sensor
    // are we within y-coordinate of the obstacle?
    if (pos_y_ < OBSTACLE_Y_2) {
      int obs_y = pos_y_;
      int obs_x = OBSTACLE_X_1;

      // calculate distance from the obstacle
      double dist = sqrt(
          pow((pos_x_ - obs_x), 2.0) + pow((pos_y_ - obs_y), 2.0));

      // too close, avoid the obstacle
      if (dist < DISTANCE_THRESHOLD) {
        //return true;
          return false;
      }
    }
    // no obstacle nearby
    return false;
  }

  /**
   * TURN LEFT RULE
   */
  int ControlAgent::ruleTurnLeft()
  {
    if (damaged){
      delta_left = CONTROL_DELTA_NEUTRAL_RECOVERY;
      delta_right = CONTROL_DELTA_ACTIVE_LEFT_RECOVERY;
      omega_left = CONTROL_OMEGA_NEUTRAL_RECOVERY;
      omega_right = CONTROL_OMEGA_ACTIVE_RIGHT_RECOVERY;
    }
    else {
      delta_left = CONTROL_DELTA_NEUTRAL;
      delta_right = CONTROL_DELTA_ACTIVE_LEFT;
      omega_left = CONTROL_OMEGA_NEUTRAL;
      omega_right = CONTROL_OMEGA_ACTIVE_RIGHT;
    }

    return RULE_TURN_LEFT;
  }

  /**
   * TURN RIGHT RULE
   */
  int ControlAgent::ruleTurnRight()
  {
    if (damaged){
      delta_left = CONTROL_DELTA_ACTIVE_LEFT_RECOVERY;
      delta_right = CONTROL_DELTA_NEUTRAL_RECOVERY;
      omega_left = CONTROL_OMEGA_ACTIVE_LEFT_RECOVERY;
      omega_right = CONTROL_OMEGA_NEUTRAL_RECOVERY;
    }
    else {
      delta_left = CONTROL_DELTA_ACTIVE_LEFT;
      delta_right = CONTROL_DELTA_NEUTRAL;
      omega_left = CONTROL_OMEGA_ACTIVE_LEFT;
      omega_right = CONTROL_OMEGA_NEUTRAL;
    }
    return RULE_TURN_RIGHT;
  }

  /**
   * GO FORWARD RULE
   */
  int ControlAgent::ruleGoForward()
  {
    if (damaged){
      delta_left = CONTROL_DELTA_ACTIVE_LEFT_RECOVERY;
      delta_right = CONTROL_DELTA_ACTIVE_RIGHT_RECOVERY;
      omega_left = CONTROL_OMEGA_ACTIVE_LEFT_RECOVERY;
      omega_right = CONTROL_OMEGA_ACTIVE_RIGHT_RECOVERY;
    }
    else {
      delta_left = CONTROL_DELTA_ACTIVE_LEFT;
      delta_right = CONTROL_DELTA_ACTIVE_RIGHT;
      omega_left = CONTROL_OMEGA_ACTIVE_LEFT;
      omega_right = CONTROL_OMEGA_ACTIVE_RIGHT;
    }

    return RULE_GO_FORWARD;
  }

  /**
   * IDLE RULE
   */
  int ControlAgent::ruleIdle()
  {
    if (damaged){
      delta_left = CONTROL_DELTA_NEUTRAL_RECOVERY;
      delta_right = CONTROL_DELTA_NEUTRAL_RECOVERY;
      omega_left = CONTROL_OMEGA_NEUTRAL_RECOVERY;
      omega_right = CONTROL_OMEGA_NEUTRAL_RECOVERY;
    }
    else {
      delta_left = CONTROL_DELTA_NEUTRAL;
      delta_right = CONTROL_DELTA_NEUTRAL;
      omega_left = CONTROL_OMEGA_NEUTRAL;
      omega_right = CONTROL_OMEGA_NEUTRAL;
    }

    return RULE_IDLE;
  }

  /**
   * Avoid Obstacle Rule
   */
  int ControlAgent::ruleAvoidObstacle()
  {
    if (damaged){
      delta_left = CONTROL_DELTA_ACTIVE_LEFT_RECOVERY;
      delta_right = CONTROL_DELTA_NEUTRAL_RECOVERY;
      omega_left = CONTROL_OMEGA_ACTIVE_LEFT_RECOVERY;
      omega_right = CONTROL_OMEGA_NEUTRAL_RECOVERY;
    }
    else {
      delta_left = CONTROL_DELTA_ACTIVE_LEFT;
      delta_right = CONTROL_DELTA_NEUTRAL;
      omega_left = CONTROL_OMEGA_ACTIVE_LEFT;
      omega_right = CONTROL_OMEGA_NEUTRAL;
    }


    return RULE_AVOID_OBSTACLE;
  }

  float ControlAgent::getDeltaLeft()
  {
    return delta_left;
  }

  float ControlAgent::getDeltaRight()
  {
    return delta_right;
  }

  float ControlAgent::getOmegaLeft()
  {
    return omega_left;
  }

  float ControlAgent::getOmegaRight()
  {
    return omega_right;
  }

  int ControlAgent::getRule()
  {
    return rule_;
  }

  const char* ControlAgent::getRuleString()
  {
    if (close_to_obstacle_){
      return rule_avoid_obstacle;
    }
    switch (rule_) {
      case RULE_IDLE:
        return rule_idle;
      case RULE_TURN_LEFT:
        return rule_turn_left;
      case RULE_TURN_RIGHT:
        return rule_turn_right;
      case RULE_GO_FORWARD:
        return rule_go_forward;
      case RULE_AVOID_OBSTACLE:
        return rule_avoid_obstacle;
    }
    return rule_idle;
  }

  double ControlAgent::getDesiredCourse()
  {
    return desired_course_;
  }

  double ControlAgent::getCourseError()
  {
    return course_err_;
  }

  vector<int> ControlAgent::getObstacleX()
  {
    return obstacle_x_;
  }

  vector<int> ControlAgent::getObstacleY()
    {
      return obstacle_y_;
    }

  bool ControlAgent::has_reached_wp()
  {
    return wp_reached_;
  }
} /* namespace std */
