/*
 * Agent.h
 *
 *  Created on: Nov 20, 2015
 *      Author: fwmav
 */

#ifndef AGENT_H_
#define AGENT_H_

#define AGENT_COLLECTION 0
#define AGENT_CONTROL 1
#define AGENT_STRATEGY 2

namespace std
{

  class Agent
  {
  public:
    // pure virtual function providing interface framework.
    virtual int step() = 0;
    int getType() {
      return type_;
    }
  protected:
     int type_;
  };

} /* namespace std */

#endif /* AGENT_H_ */
