#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <fstream>
#include <queue>
#include <math.h>

#define PI 3.14159265358979323846

#include "Dot.h"
#include "Event.h"
#include "serverthread.h"
#include "CollectionAgent.h"
#include "StrategyAgent.h"
#include "ControlAgent.h"

//#define CONTROL_DEG_THRESHOLD 20 //deg
//#define CONTROL_DELTA_ACTIVE_LEFT 10
//#define CONTROL_DELTA_ACTIVE_RIGHT 10
//#define CONTROL_DELTA_NEUTRAL 0
//#define CONTROL_OMEGA_NEUTRAL 12
//#define CONTROL_OMEGA_ACTIVE_LEFT 25
//#define CONTROL_OMEGA_ACTIVE_RIGHT 30

//#define WP_NB 2 // number of waypoints
//#define WP_1_X 100 // px
//#define WP_1_Y 250 // px
//#define WP_2_X 500 // px
//#define WP_2_Y 250 // px

//#define DISTANCE_THRESHOLD 70 // px

// waypoints list
const static int wp_x[] = {WP_1_X, WP_2_X, WP_3_X, WP_4_X, WP_5_X};
const static int wp_y[] = {WP_1_Y, WP_2_Y, WP_3_Y, WP_4_Y, WP_5_Y};

namespace Ui {
  class Dialog;
}

class Dialog : public QDialog {
  Q_OBJECT

public:
  static const int WEBCAM_INDEX = 1;

  explicit Dialog(QWidget *parent = 0);
  ~Dialog();


  // This function converts decimal degrees to radians
  static double deg2rad(double deg) {
    return (deg * PI / 180);
  }

  //  This function converts radians to decimal degrees
  static double rad2deg(double rad) {
    return (rad * 180 / PI);
  }

private:
  Ui::Dialog *ui;

  ServerThread t;

  queue<Event> fifo;
  queue<Agent*> agent_schedule;

  CollectionAgent collection_agent;
  StrategyAgent strategy_agent;
  ControlAgent control_agent;

  cv::VideoCapture cap_webcam;
  cv::VideoWriter output_video;
  cv::VideoWriter output_video_raw;

  std::ofstream datafile;

  cv::Mat mat_original;
  cv::Mat mat_raw;
  cv::Mat mat_gray;
  cv::Mat mat_processed;
  cv::Mat threshold;

  // for undistortion of images
  cv::Size image_size;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;

  Dot dot_front;  // front of the vehicle
  Dot dot_center; // corner marker close to the center
  Dot dot_right;  // left side marker

  vector<Dot> dots_filtered; // for object orientation finding

  Point vehicle_pos; // center of the vehicle
  double vehicle_orientation; // orientation of the vehicle

  Point wp[WP_NB];
  int wp_idx;

  int rule;
  const char* rule_string;

  QImage qimgOriginal;
  QTimer* tmrTimer;
  QTimer* thdTimer;

  bool recording;
  bool connected;
  bool object_found;

  struct timespec startup_time;
  struct timespec now;
  float time_sec;
  double phi;
  float delta_left;
  float delta_right;
  float omega_left;
  float omega_right;
  double cosphi;
  double desired_course;

  // initial and max values of the parameters of interests.
  const static int cannyThresholdInitialValue = 119;
  const static int accumulatorThresholdInitialValue = 50;
  const static int maxAccumulatorThreshold = 200;
  const static int maxCannyThreshold = 255;
  const static int minRadiusDefault = 39;
  const static int maxRadiusDeafault = 60;
  // distance between center dot and center of the vehicle
  const static int centerDotToCGpx = 20;

  //initial min and max HSV filter values.
  const static int H_MIN_Default = 0;
  const static int H_MAX_Default = 256;
  const static int S_MIN_Default = 55;
  const static int S_MAX_Default = 256;
  const static int V_MIN_Default = 183;
  const static int V_MAX_Default = 251;



  //declare and initialize both parameters that are subjects to change
  int canny_threshold;
  int accumulator_threshold;
  int min_rad;
  int max_rad;

  int H_MIN;
  int H_MAX;
  int S_MIN;
  int S_MAX;
  int V_MIN;
  int V_MAX;

  // auxilliary
  bool calculatePosition();
  void displayData();
  float getTime();
  void scheduleAgents();
  void visualiseVehicle();
  void updateControl();
  void executeSchedule();

  // gui
  void displayResults();

  // datalog
  void writeHeader();
  void saveData();

  // vehicle detection
  void detectRobot();
  bool trackFilteredObject(cv::Mat threshold);
  //  void trackFilteredObject(Dot &dot, cv::Mat threshold); // TODO: obsolete
  void morphOps(cv::Mat &thresh);
  void HsvDetection();
  void drawObject(vector<Dot> dots, cv::Mat &frame);
  bool getFCR(vector<Dot> &dots);

public slots:
  void processFrameAndUpdateGui();
  void processIncomingConnections();

private slots:
  void on_btnLocate_clicked();
  void on_btnRecordOrStop_clicked();
  void on_slideOmegaLeft_valueChanged(int value);
  void on_slideOmgeraRight_valueChanged(int value);
  void on_slideDeltaleft_valueChanged(int value);
  void on_slideDeltaRight_valueChanged(int value);
  void on_pushButton_clicked();
  void on_btnInit_clicked();
  void on_btnParams_clicked();
  void on_btnStop_clicked();
  void on_btnQuit_clicked();
};

#endif // DIALOG_H
