#ifndef SERVERTHREAD_H
#define SERVERTHREAD_H

#include <QThread>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write
#include <string>
#include <math.h>
#include <iostream>
#include <assert.h>
#include <fcntl.h>

enum ServerStatus {
  IDLE,
  SEND_PARAMS,
  SEND_INIT,
  SEND_STOP,
  SEND_QUIT
};

class ServerThread: public QThread
{
  //Q_OBJECT
private:
  void run();
  void server_init();
  void server_connect();
  void server_send();
  int check_for_hi();
  int check_for_init();
  int check_for_stop();
  int check_for_params();
  int check_for_quit();

public:
  // defines
  static const int SERVER_MSG_SIZE = 64;
  static const int SERVER_PORT = 25566;
  static const float DEFAULT_DELTA_RAD = 6.28;
  static const float DEFAULT_OMEGA_RAD = 12.56;
  static const int LEFT = 1;
  static const int RIGHT = 0;

  static const char hi[8];
  static const char init[8];
  static const char params[8];
  static const char stop[8];
  static const char quit[8];

  // control variables
  float rx_delta_left;
  float rx_delta_right;
  float rx_omega_left;
  float rx_omega_right;

  float tx_delta_left;
  float tx_delta_right;
  float tx_omega_left;
  float tx_omega_right;

  bool stop_server;

  // connection variables
  int socket_desc;
  int new_socket;
  int c;
  int loop_flag;
  int send_flag;
  struct sockaddr_in server;
  struct sockaddr client;
  char client_message[SERVER_MSG_SIZE];
  char server_message[SERVER_MSG_SIZE];
  int read_size;
  struct timeval tv;

  //server status
  enum ServerStatus status;
};

#endif // SERVERTHREAD_H
