// Code written by Helena B McCurdy

#include "Dot.h"


Dot::Dot(void)
{
  setXPos(0);
  setYPos(0);
}


// Dot constructor
// Sets the HSV values for each colour needed
Dot::Dot(String colour){
  if(colour == "orange"){
      setColour("orange");
      setType('R');
      setHSVmin(Scalar(0,135,95));
      setHSVmax(Scalar(89,256,256));

    } else if(colour == "green"){
      setColour("green");
      setType('F');
      setHSVmin(Scalar(40,139,0));
      setHSVmax(Scalar(93,219,179));

    } else if(colour == "pink"){
      setColour("pink");
      setType('B');
      setHSVmin(Scalar(149,77,96));
      setHSVmax(Scalar(177,209,219));

    } else if(colour == "blue"){
      //Not using now
      setColour("blue");
      setType('B');
      setHSVmin(Scalar(95,19,168));
      setHSVmax(Scalar(132,197,251));

    } else if(colour == "yellow"){
      setColour("yellow");
      setType('F');
      setHSVmin(Scalar(0,20,191));
      setHSVmax(Scalar(70,256,256));

    }

  setXPos(0);
  setYPos(0);

}

Dot::~Dot()
{
}

String Dot::getColour(){
  return Dot::colour;
}

void Dot::setColour(String c){
  Dot::colour = c;
}

int Dot::getXPos(){
  return xPos;
}

void Dot::setXPos(int x){
  xPos = x;
}

int Dot::getYPos(){
  return yPos;
}

void Dot::setYPos(int y){
  yPos = y;
}

void Dot::setType(char t){
  type = t;
}

char Dot::getType(){
  return type;
}

Scalar Dot::getHSVmin(){
  return Dot::HSVmin;
}

Scalar Dot::getHSVmax(){
  return Dot::HSVmax;
}

void Dot::setHSVmin(Scalar min){
  Dot::HSVmin = min;
}

void Dot::setHSVmax(Scalar max){
  Dot::HSVmax = max;
}

