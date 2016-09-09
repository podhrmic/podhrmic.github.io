// Code written by Helena B McCurdy
#include <string>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
class Dot
{
public:
  Dot(void);
  ~Dot(void);

  Dot(String);

  int getXPos();
  int getYPos();
  char getType();
  String getColour();
  void setColour(String);
  void setXPos(int);
  void setYPos(int);
  void setType(char);
  Scalar getHSVmin();
  Scalar getHSVmax();
  void setHSVmin(Scalar min);
  void setHSVmax(Scalar max);

private:
  int xPos, yPos;
  char type;
  String colour;
  cv::Scalar HSVmin;
  cv::Scalar HSVmax;
};

