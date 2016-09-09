#include "dialog.h"
#include "ui_dialog.h"
#include <QtCore>
#include <QTextStream>

#define DEBUG 0
#define DEMO_VIDEO 0
#define CALIB 0

using namespace cv;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be tracked in frame (3: one for each dot being used)
const int MAX_NUM_OBJECTS = 3;
//minimum and maximum object area -> smaller = more chance of noise
const int MIN_OBJECT_AREA = 15 * 15;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;
const int TIMER_PERIOD_MS = 33;

/**
 * @brief Dialog::Dialog
 * @param parent
 */
Dialog::Dialog(QWidget *parent) :
    QDialog(parent), ui(new Ui::Dialog)
{
  ui->setupUi(this);
  dot_front = Dot();
  dot_center = Dot();
  dot_right = Dot();
  time_sec = 0.0;
  phi = 0.0;
  delta_left = 0.0;
  delta_right = 0.0;
  omega_left = 0.0;
  omega_right = 0.0;

  object_found = false;

  rule = 0;
  rule_string = "-";

  // initial orientation
  vehicle_pos = Point(1, 1);
  vehicle_orientation = 0;

  //control variables
  desired_course = 0;

  // waypoint initialization
  for (int i = 0; i < WP_NB; i++) {
    wp[i] = Point(wp_x[i], wp_y[i]);
  }
  wp_idx = 0;

  // open video
  if (DEMO_VIDEO) {
    cap_webcam.open("demo.avi");
  } else {
    cap_webcam.open(WEBCAM_INDEX);
  }

  // check if success
  if (cap_webcam.isOpened() == false) {
    ui->txtConsole->appendPlainText("Camera not opened");
    return;
  }

  // load calibration file
  if (CALIB) {
    FileStorage fs("calib_data.xml", FileStorage::READ);
    fs["image_Width"] >> image_size.width;
    fs["image_Height"] >> image_size.height;
    fs["Camera_Matrix"] >> camera_matrix;
    fs["Distortion_Coefficients"] >> dist_coeffs;
  }

  // connect timer at 30Hz
  tmrTimer = new QTimer(this);
  connect(tmrTimer, SIGNAL(timeout()), this, SLOT(processFrameAndUpdateGui()));
  tmrTimer->start(TIMER_PERIOD_MS);  // we should use threads instead...

  // connect connection timer at 30Hz
  thdTimer = new QTimer(this);
  connect(thdTimer, SIGNAL(timeout()), this,
      SLOT(processIncomingConnections()));
  thdTimer->start(TIMER_PERIOD_MS);

  // set initial flags
  recording = false;
  connected = false;

  //declare and initialize both parameters that are subjects to change
  canny_threshold = cannyThresholdInitialValue;
  accumulator_threshold = accumulatorThresholdInitialValue;
  min_rad = minRadiusDefault;
  max_rad = maxRadiusDeafault;

  H_MIN = H_MIN_Default;
  H_MAX = H_MAX_Default;
  S_MIN = S_MIN_Default;
  S_MAX = S_MAX_Default;
  V_MIN = V_MIN_Default;
  V_MAX = V_MAX_Default;

  //initialize agents
  collection_agent = CollectionAgent(&fifo);
  strategy_agent = StrategyAgent(&fifo);
  control_agent = ControlAgent(&fifo);

  // initialize with first waypoint
  control_agent.set_wp(strategy_agent.get_wp_x(), strategy_agent.get_wp_x());

  t.start();
}

Dialog::~Dialog()
{
  if (output_video_raw.isOpened()) {
    output_video_raw.release();
  }
  if (output_video.isOpened()) {
    output_video.release();
  }
  if (datafile.is_open()) {
    datafile.close();
  }
  delete ui;
}

/**
 * @brief Dialog::processIncomingConnections
 * Update sliders with received numbers
 * The values are not updates until they are received
 * from the robot, as a visible verification of the
 * connection
 */
void Dialog::processIncomingConnections()
{
  delta_left = t.rx_delta_left;
  ui->labelDeltaLeftVal->setText(QString::number(delta_left) + "[rad/s]");
  delta_right = t.rx_delta_right;
  ui->labelDeltaRightVal->setText(QString::number(delta_right) + "[rad/s]");
  omega_left = t.rx_omega_left;
  ui->labelOmgLeftVal->setText(QString::number(omega_left) + "[rad/s]");
  omega_right = t.rx_omega_right;
  ui->labelOmrRightVal->setText(QString::number(omega_right) + "[rad/s]");
}

/**
 * Detect robot in the image
 * 1) HSV detection of the markers
 * 2) triangulate and decide the front and side markers
 * 3) if successful, calculate position and orientation
 * 4) is successful, mark object_found == TRUE
 * 5) highlight in image
 */
void Dialog::detectRobot()
{
  // reset the flag
  object_found = false;

  // detect the color dot
  HsvDetection();

  // check if we found 3 markers
  if (trackFilteredObject(threshold)) {
    // if object found, get orientation and position
    if (getFCR(dots_filtered)) {
      // calculate position & azimuth
      object_found = calculatePosition();

      // add event into fifo
      if (object_found) {
        fifo.push(Event(EVENT_NEW_DATA));
      }
    }
  }
}

/**
 * Display the vehicle in the image
 */
void Dialog::visualiseVehicle(){
    if (DEBUG) {
      drawObject(dots_filtered, mat_original);
    }

    circle(mat_original, Point(dot_center.getXPos(), dot_center.getYPos()), 3,
        Scalar(255, 0, 0), -1, 8, 0);
    circle(mat_original, Point(vehicle_pos.x, vehicle_pos.y), 3,
        Scalar(0, 255, 0), -1, 8, 0);

    // make line to front
    line(mat_original, Point2f(dot_center.getXPos(), dot_center.getYPos()),
        Point2f(dot_front.getXPos(), dot_front.getYPos()), Scalar(0, 255, 0),
        1);

    // show waypoint
    circle(mat_original, Point(strategy_agent.get_wp_x(), strategy_agent.get_wp_y()),
        3, Scalar(150, 255, 0), -1, 8, 0);

    /// show startpoint
    circle(mat_original, Point(150, 250),
        3, Scalar(0, 255, 255), -1, 8, 0);


    // show rule
    putText(mat_original, rule_string, cvPoint(30, 30),
        FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250, 250, 250), 1, CV_AA);

    // show obstacle
#if USE_OBSTACLE
    vector<int> obst_x = control_agent.getObstacleX();
    vector<int> obst_y = control_agent.getObstacleY();
    line(mat_original, Point2f(obst_x.at(0), obst_y.at(0)),
        Point2f(obst_x.at(1), obst_y.at(1)), Scalar(0, 255, 0),
        2);
#endif
}

/**
 * @brief Dialog::HsvDetection
 * Filter frame with HSV threshold values
 */
void Dialog::HsvDetection()
{
  // convert to HSV
  cvtColor(mat_original, mat_processed, COLOR_BGR2HSV);

  // filter according to values in range
  inRange(mat_processed, Scalar(H_MIN, S_MIN, V_MIN),
      Scalar(H_MAX, S_MAX, V_MAX), threshold);

  // dillate / erode
  morphOps(threshold);
}

/**
 * @brief Dialog::trackFilteredObject
 * @param threshold
 * @param cameraFeed
 */
bool Dialog::trackFilteredObject(Mat threshold)
{
  dots_filtered.clear();
  Mat temp;
  threshold.copyTo(temp);
  //these two vectors needed for output of findContours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  //find contours of filtered image using openCV findContours function
  findContours(temp, contours, hierarchy, CV_RETR_CCOMP,
      CV_CHAIN_APPROX_SIMPLE);
  //use moments method to find our filtered object

  if (hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
    //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
    if (numObjects < MAX_NUM_OBJECTS + 1) {
      for (int index = 0; index >= 0; index = hierarchy[index][0]) {

        Moments moment = moments((cv::Mat) contours[index]);
        double area = moment.m00;
        //if the area is less than 20 px by 20px then it is probably just noise
        //if the area is the same as the 3/2 of the image size, probably just a bad filter
        //we only want the object with the largest area so we safe a reference area each
        //iteration and compare it to the area in the next iteration.

        if (area > MIN_OBJECT_AREA) {
          //Shape class that updates x and y not create a new object every time
          Dot dot;
          dot.setXPos(moment.m10 / area);
          dot.setYPos(moment.m01 / area);
          dots_filtered.push_back(dot);
        }
      }
      // object found
      return true;
    }
  }
  // no object found
  return false;
}

/**
 * This method determines which dot is the front, which is center, and which is the right.
 * It will set their type respectively.
 *
 * Use only with same color markers
 */
bool Dialog::getFCR(vector<Dot> &dots)
{
  //check if three dots in vector
  if (dots.size() == 3) {
    Dot &dot1 = dots[0];
    Dot &dot2 = dots[1];
    Dot &dot3 = dots[2];

    double length12 = sqrt(
        pow((double) (dot1.getXPos() - dot2.getXPos()), 2)
            + pow((double) (dot1.getYPos() - dot2.getYPos()), 2));
    double length23 = sqrt(
        pow((double) (dot2.getXPos() - dot3.getXPos()), 2)
            + pow((double) (dot2.getYPos() - dot3.getYPos()), 2));
    double length31 = sqrt(
        pow((double) (dot3.getXPos() - dot1.getXPos()), 2)
            + pow((double) (dot3.getYPos() - dot1.getYPos()), 2));

    //check which is longest
    if (length12 > length23 && length12 > length31) {
      //12 is longest therefore 3 is center
      dot3.setType('C');
      dot_center = dot3;
      if (length23 < length31) {
        //23 is shortest therfore 2 is right and 1 is front
        dot2.setType('R');
        dot_right = dot2;
        dot1.setType('F');
        dot_front = dot1;
      } else {
        //12 is shortest therfore 1 is right and 2 is front
        dot2.setType('F');
        dot_front = dot2;
        dot1.setType('R');
        dot_right = dot1;
      }
    } else if (length23 > length12 && length23 > length31) {
      //23 is longest therefore 1 is center
      dot1.setType('C');
      dot_center = dot1;
      if (length31 < length12) {
        //31 is shortest therfore 3 is right and 2 is front
        dot3.setType('R');
        dot_right = dot3;
        dot2.setType('F');
        dot_front = dot2;
      } else {
        //12 is shortest therfore 2 is right and 3 is front
        dot3.setType('F');
        dot_front = dot3;
        dot2.setType('R');
        dot_right = dot2;
      }

    } else if (length31 > length12 && length31 > length23) {
      //31 is the longest therefore 2 is center
      dot2.setType('C');
      dot_center = dot2;
      if (length23 < length12) {
        //23 is shortest therfore 3 is right and 1 is front
        dot3.setType('R');
        dot_right = dot3;
        dot1.setType('F');
        dot_front = dot1;
      } else {
        //12 is shortest therfore 1 is right and 3 is front
        dot3.setType('F');
        dot_front = dot3;
        dot1.setType('R');
        dot_right = dot1;
      }
    }
    return true;
  }
  return false;
}

/**
 * Schedule and step agents as necessary
 */
void Dialog::scheduleAgents()
{
  static bool wp_requested = false;

  // read events fifo
  if (!fifo.empty()) {
    Event evt1 = fifo.front();
    fifo.pop();

    qDebug() << "Control agent distance = " << control_agent.getDistance();

    if (control_agent.has_reached_wp() && !wp_requested) {
        // schedule Strategy agent
        agent_schedule.push((Agent*)&strategy_agent);
        wp_requested = true;
        qDebug() << "Control agent WP REACHED";
      }


    switch(evt1.get_type()) {
      case EVENT_NEW_DATA:
        // process new raw data
        // provide new data to collection agent
        collection_agent.set_pos(vehicle_pos.x, vehicle_pos.y, phi);
        // prepare the agent to be stepped
        agent_schedule.push((Agent*)&collection_agent);
        qDebug() << "EVENT_NEW_DATA";
        break;
      case EVENT_NEW_FILTERED_DATA:
        //update vehicle states
        vehicle_pos.x = collection_agent.get_pos_x();
        vehicle_pos.y = collection_agent.get_pos_y();
        vehicle_orientation = collection_agent.get_heading();
        // update controller agent
        control_agent.set_pos(vehicle_pos.x, vehicle_pos.y, vehicle_orientation);
        // update control agent
        control_agent.set_wp(strategy_agent.get_wp_x(), strategy_agent.get_wp_y());
        //prepare agent to be stepped
        agent_schedule.push((Agent*)&control_agent);
        // check the server status and update the status
        control_agent.server_status = t.status;
        // update time
        control_agent.cur_time = time_sec;

        qDebug() << "EVENT_NEW_FILTERED_DATA";
        break;
      case EVENT_WP_REACHED:
        // wp reached, get new waypoint
        // schedule Strategy agent
        //agent_schedule.push((Agent*)&strategy_agent);
        qDebug() << "EVENT_WP_REACHED";
        break;
      case EVENT_NEW_WP:
        // new wp available
        // update control agent
        control_agent.set_wp(strategy_agent.get_wp_x(), strategy_agent.get_wp_y());
        //prepare agent to be stepped
        agent_schedule.push((Agent*)&control_agent);
        qDebug() << "EVENT_NEW_WP: x=" << control_agent.get_wp_x() << ", y=" << control_agent.get_wp_y();
        wp_requested = false;
        break;
      case EVENT_RULE_THAT_FIRED:
        // update the records of the rules
        rule = control_agent.getRule();
        rule_string = control_agent.getRuleString();
        qDebug() << "EVENT_RULE_THAT_FIRED";
        break;
      case EVENT_CONTROL_UPDATE:
        // new controls available
        // update control system
        updateControl();
        qDebug() << "EVENT_CONTROL_UPDATE";
        break;
    }
  }
}

/**
 * Steps the agents that were scheduled
 */
void Dialog::executeSchedule() {
  qDebug() << "Queue size = " << agent_schedule.size();
  if (!agent_schedule.empty()) {
    agent_schedule.front()->step();
    agent_schedule.pop();
  }
}

void Dialog::updateControl(){
  t.tx_delta_left = control_agent.getDeltaLeft();
  t.tx_delta_right = control_agent.getDeltaRight();
  t.tx_omega_left = control_agent.getOmegaLeft();
  t.tx_omega_right = control_agent.getOmegaRight();
  ui->labelStatus->setText(rule_string);
}

/**
 * @brief Dialog::processFrameAndUpdateGui
 * Essentially the main loop
 */
void Dialog::processFrameAndUpdateGui()
{
  // get new frame
  cap_webcam.read(mat_original);

  // safeguard for no new video
  if (mat_original.empty())
    return;
  // else continue processing

  // undistort
  if (!camera_matrix.empty() && CALIB) {
    Mat temp = mat_original.clone();
    undistort(temp, mat_original, camera_matrix, dist_coeffs);
  }

  if (recording) {
    // save a copy of mat original
    mat_raw = mat_original.clone();

    // locate robot
    detectRobot();

    if (object_found) {
      // get current time
      time_sec = getTime();
      if (DEBUG) {
        qDebug() << "Time[s]: " << time_sec;
      }

      // Scheduler
      scheduleAgents();

      // Execute
      executeSchedule();

      //append console
      displayData();

      // show progress
      visualiseVehicle();

      // save data
      saveData();
    }

    // save video
    output_video.write(mat_original);
    output_video_raw.write(mat_raw);
  }

  // Display results
  displayResults();
}

/**
 * Display results in GUI
 */
void Dialog::displayResults()
{
  // displays the picture
  cv::cvtColor(mat_original, mat_original, CV_BGR2RGB);
  QImage qimgOriginal((uchar*) mat_original.data, mat_original.cols,
      mat_original.rows, mat_original.step, QImage::Format_RGB888);
  ui->imgCamera->setPixmap(QPixmap::fromImage(qimgOriginal));
}


/**
 * @brief Dialog::saveData
 */
void Dialog::saveData()
{
  datafile << time_sec << ", " << vehicle_pos.x << ", " << vehicle_pos.y << ", "
      << vehicle_orientation << ", " << delta_left << ", " << delta_right
      << ", " << omega_left << ", " << omega_right << ", " << rule << ", "
      << control_agent.getDesiredCourse() << ", " << control_agent.getCourseError()
      << control_agent.getRule() << ", " << control_agent.server_status << ", "
      << control_agent.agent_status << ", " << control_agent.final_time << ", "
      << control_agent.d_orientation << ", " << control_agent.damaged << ", "
      << endl;
}

/**
 * @brief Dialog::displayData
 */
void Dialog::displayData()
{
  QString str;
  str.append("Time[s]: ");
  str.append(QString::number(time_sec));
  str.append(", x[px]: ");
  str.append(QString::number(vehicle_pos.x));
  str.append(", y[px]: ");
  str.append(QString::number(vehicle_pos.y));
  str.append(", phi[deg]: ");
  str.append(QString::number(vehicle_orientation));
  str.append(", desired_course[deg]: ");
  str.append(QString::number(desired_course));
  str.append(", rule: ");
  str.append(QString::number(rule));
  str.append(", server_status: ");
  str.append(QString::number(control_agent.server_status));
  str.append(", agent_status: ");
  str.append(QString::number(control_agent.agent_status));
  str.append(", turn_time: ");
  str.append(QString::number(control_agent.final_time));
  str.append(", d_orientation: ");
  str.append(QString::number(control_agent.d_orientation));
  str.append(", damaged: ");
  str.append(QString::number(control_agent.damaged));
  ui->txtConsole->appendPlainText(str);
}

/**
 * @brief Dialog::getTime
 * Return current time
 */
float Dialog::getTime()
{
  clock_gettime(CLOCK_MONOTONIC, &now);
  /* time difference to startup */
  time_t d_sec = now.tv_sec - startup_time.tv_sec;
  long d_nsec = now.tv_nsec - startup_time.tv_nsec;

  /* wrap if negative nanoseconds */
  if (d_nsec < 0) {
    d_sec -= 1;
    d_nsec += 1000000000L;
  }

  return ((double) d_sec + ((double) d_nsec) / 1000000000.0);
}

/**
 * @brief Dialog::calculatePosition
 */
bool Dialog::calculatePosition()
{
  if (DEBUG) {
    qDebug() << "center x: " << dot_center.getXPos();
    qDebug() << "center y: " << dot_center.getYPos();
  }

  Point u = Point(1, 0);
  Point v = Point(dot_front.getXPos() - dot_center.getXPos(),
      dot_front.getYPos() - dot_center.getYPos());
  if (DEBUG) {
    qDebug() << "u = (" << u.x << ", " << u.y << ")";
    qDebug() << "v = (" << v.x << ", " << v.y << ")";
  }

  cosphi = u.x * v.x + u.y * v.y;
  if (DEBUG) {
    qDebug() << "u * v = " << cosphi;
  }

  cosphi = cosphi / (sqrt(u.x * u.x + u.y * u.y) * sqrt(v.x * v.x + v.y * v.y));
  if (DEBUG) {
    qDebug() << "Intermittent product = " << cosphi;
  }

  phi = rad2deg(acos(cosphi));
  // check sign
  if (v.y < 0) {
    phi = -1 * phi;
  }

  // update CG
  vehicle_pos.x = (cos(deg2rad(phi)) * centerDotToCGpx) + dot_center.getXPos();
  vehicle_pos.y = (sin(deg2rad(phi)) * centerDotToCGpx) + dot_center.getYPos();

  if (DEBUG) {
    qDebug() << "phi = " << phi << "[deg]";
    qDebug() << "vehicle_pos = (" << vehicle_pos.x << ", " << vehicle_pos.y
        << ")";
  }

  // check for singularities
  if (isnan(phi)) {
    return false;
  } else {
    return true;
  }
}

/**
 * @brief Dialog::morphOps
 * @param thresh
 * Dillate and Erode image matrix
 */
void Dialog::morphOps(Mat &thresh)
{
  //create structuring element that will be used to "dilate" and "erode" image.
  //the element chosen here is a 3px by 3px rectangle
  Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
  //dilate with larger element so make sure object is nicely visible
  Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

  erode(thresh, thresh, erodeElement);
  erode(thresh, thresh, erodeElement);

  dilate(thresh, thresh, dilateElement);
  dilate(thresh, thresh, dilateElement);
}

/**
 * Display color markers
 *
 * @brief drawObject
 * @param dots to be drawn
 * @param frame to drawn to
 */
void Dialog::drawObject(vector<Dot> dots, Mat &frame)
{
  for (vector<Dot>::iterator it = dots.begin(); it != dots.end(); ++it) {
    Dot tmp = *it;
    char type = tmp.getType();
    cv::Scalar color;
    switch (type) {
      case 'F':
        color = Scalar(0, 255, 0);
        break;
      case 'C':
        color = Scalar(255, 0, 0);
        break;
      case 'R':
        color = Scalar(0, 0, 255);
        break;
      default:
        color = Scalar(128, 128, 128);
        break;
    }

    cv::circle(frame, cv::Point(tmp.getXPos(), tmp.getYPos()), 10, color, 3);
  }
}

/**
 * @brief Dialog::on_btnLocate_clicked
 */
void Dialog::on_btnLocate_clicked()
{
  //locate the robot in the airfame
  ui->txtConsole->appendPlainText("Stopping...");
  t.stop_server = true;
}

/**
 * @brief Dialog::on_btnRecordOrStop_clicked
 */
void Dialog::on_btnRecordOrStop_clicked()
{
  if (recording == true) {
    //start recoring
    ui->btnRecordOrStop->setText("Record");
    if (output_video.isOpened() || datafile.is_open()) {
      output_video.release();
      output_video_raw.release();
      datafile.close();
      ui->txtConsole->appendPlainText("Stopped.");
    } else {
      ui->txtConsole->appendPlainText("Error releasing video.");
    }
    recording = false;
  } else {
    //start recoring
    ui->btnRecordOrStop->setText("Stop");
    Size S = Size((int) cap_webcam.get(CV_CAP_PROP_FRAME_WIDTH),  // Acquire input size
    (int) cap_webcam.get(CV_CAP_PROP_FRAME_HEIGHT));
    time_t f_seconds;

    // Filename video
    char logname[32];
    f_seconds = time(NULL);
    strftime(logname, 32, "%F_%H-%M-%S.avi", localtime(&f_seconds));
    const char *s = logname;
    String name;
    name.assign(s, strlen(s));
    output_video.open(name, CV_FOURCC('M', 'J', 'P', 'G'), 30, S, true);

    // Filename raw data
    char logname_raw[32];
    strftime(logname_raw, 32, "%F_%H-%M-%S_raw.avi", localtime(&f_seconds));
    const char *s_raw = logname_raw;
    String name_raw;
    name_raw.assign(s_raw, strlen(s_raw));
    output_video_raw.open(name_raw, CV_FOURCC('M', 'J', 'P', 'G'), 30, S, true);

    // Open textfile
    strftime(logname, 32, "%F_%H-%M-%S.csv", localtime(&f_seconds));
    datafile.open(logname);

    if (output_video.isOpened() && datafile.is_open()) {
      ui->txtConsole->appendPlainText("Recording...");
      clock_gettime(CLOCK_MONOTONIC, &startup_time);
      writeHeader();
      recording = true;
    } else {
      ui->txtConsole->appendPlainText("Error recording.");
      recording = false;
    }
  }
}

/**
 * Writes header to the datafile
 * Time - elapsed time [s]
 * x - position [px]
 * y - position [px]
 * phi - orientation [deg]
 *
 * Orientation:
 *          /|\ -90 deg
 *           |
 *           |
 *           |
 * -180 deg  |
 * --------------------> 0 deg
 *  180 deg  |
 *           |
 *           |
 *           |
 *          \|/ +90deg
 *
 * delta_left [rad/s] - zero is neutral,
 * -omega/2 < delta < omega/2
 * delta_right [rad/s]
 * omega_left [rad/s]
 * omega_right [rad/s]
 * rule - that fired
 */
void Dialog::writeHeader()
{
  datafile
      << "Time[s], x[px], y[px], phi[deg], delta_left, "
      << "delta_right, omega_left, omega_right, rule_number, "
      << "desired_course, course_error, rule, server_status, agent_status,"
      << "turn_time, d_orientation, damaged,"
      << "\n";
}

/**
 * Omega Left Slider
 * @brief Dialog::on_slideOmegaLeft_valueChanged
 * @param value
 */
void Dialog::on_slideOmegaLeft_valueChanged(int value)
{
  t.tx_omega_left = (float) value;
  qDebug() << "omega left Value: " << value;
}

/**
 * Omega Right Slider
 * @brief Dialog::on_slideOmgeraRight_valueChanged
 * @param value
 */
void Dialog::on_slideOmgeraRight_valueChanged(int value)
{
  t.tx_omega_right = (float) value;
  qDebug() << "omega right Value: " << value;
}

/**
 * Delta Left Slider
 * @brief Dialog::on_slideDeltaleft_valueChanged
 * @param value
 */
void Dialog::on_slideDeltaleft_valueChanged(int value)
{
  t.tx_delta_left = (float) value;
  qDebug() << "delta left Value: " << value;
}

/**
 * Delta Right Slider
 * @brief Dialog::on_slideDeltaRight_valueChanged
 * @param value
 */
void Dialog::on_slideDeltaRight_valueChanged(int value)
{
  t.tx_delta_right = (float) value;
  qDebug() << "delta right Value: " << value;
}

void Dialog::on_btnInit_clicked()
{
  t.status = SEND_INIT;
}

void Dialog::on_btnParams_clicked()
{
  t.status = SEND_PARAMS;
}

void Dialog::on_btnStop_clicked()
{
  t.status = SEND_STOP;
}

void Dialog::on_btnQuit_clicked()
{
  t.status = SEND_QUIT;
}

void Dialog::on_pushButton_clicked()
{
  //dummy
}
