//============================================================================
// Name        : MultiAgentSystemTest.cpp
// Author      : Michal Podhradsky, podhrad@pdx.edu
// Copyright   : MIT
//============================================================================

#include <iostream>
#include <queue>          // std::queue
using namespace std;

#include "Triangle.h"
#include "Rectangle.h"

#include "CollectionAgent.h"
#include "Event.h"

#define EVENT_NEW_DATA 0
#define EVENT_NEW_WP 1

int main() {
  queue<Event> fifo;

  Event ev1 = Event(EVENT_NEW_DATA, 13);
  fifo.push(ev1);
  fifo.push(Event(EVENT_NEW_WP,2));

  Event ev2 = fifo.front();
  cout << "First element type: " << ev2.get_type() << endl;
  fifo.pop();

  cout << "Second element type: " << fifo.front().get_type() << endl;
  fifo.pop();

  CollectionAgent ag1;

  ag1.step();
  ag1.step();
  ag1.step();

  /*
  Rectangle Rect;
  Triangle  Tri;

  Rect.setWidth(5);
  Rect.setHeight(7);
  // Print the area of the object.
  cout << "Total Rectangle area: " << Rect.getArea() << endl;

  Tri.setWidth(5);
  Tri.setHeight(7);
  // Print the area of the object.
  cout << "Total Triangle area: " << Tri.getArea() << endl;
  */

  return 0;
}
