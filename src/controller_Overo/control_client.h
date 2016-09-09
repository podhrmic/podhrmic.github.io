/*
 * Author - Hermanus Botha - Wright State University - July 4 2015
 * Modified by Michal Podhradsky, Portland State University, podhrad@pdx.edu, October 2015
 *
 * Basic communication for Gumstix Overo
 */

#ifndef CONTROL_CLIENT_H
#define CONTROL_CLIENT_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <inttypes.h>
#include <time.h>

#include "vehiclecontrol.h"

#define DEFAULT_DELTA_RAD 6.28
#define DEFAULT_OMEGA_RAD 12.56

#define LEFT 1
#define RIGHT 0

#define SERVER_ADDR "192.168.2.3"
#define SERVER_PORT 25565
#define SERVER_MSG_SIZE 64
#define MSLEEP_TIME 500
#define NSLEEP_TIME MSLEEP_TIME*1000000L

#define DEBUG 0

enum VehStatus {
  VEH_UNINIT,
  VEH_STOP,
  VEH_INIT
};

struct Vehicle {
  // state
  enum VehStatus state;

  // control params
  float delta_left_rad;
  float delta_right_rad;
  float omega_left_rad;
  float omega_right_rad;

  float rx_delta_left_rad;
  float rx_delta_right_rad;
  float rx_omega_left_rad;
  float rx_omega_right_rad;
  int new_cmd;

  // connection
  int cnt;
  int flag;
  int socket_desc;
  struct sockaddr_in server;

  // message buffers
  char message[SERVER_MSG_SIZE];
  char server_reply[SERVER_MSG_SIZE];
};

void stop_wings(void);
void init_vehicle(void);
int connect_server(void);
int check_for_init(void);
int check_for_stop(void);
int check_for_params(void);
int check_for_quit(void);
int send_hi(void);

static const char hi[8];
static const char init[8];
static const char stop[8];
static const char quit[8];

#endif /* CONTROL_CLIENT_H */
