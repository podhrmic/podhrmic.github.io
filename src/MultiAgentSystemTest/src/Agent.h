/*
 * Agent.h
 *
 *  Created on: Nov 20, 2015
 *      Author: Michal Podhradsky, podhrad@pdx.edu
 */

#ifndef AGENT_H_
#define AGENT_H_

namespace std
{

  class Agent
  {
  public:
    // pure virtual function providing interface framework.
    virtual int step() = 0;
  protected:
     int id;
  };

} /* namespace std */

#endif /* AGENT_H_ */
