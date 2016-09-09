#include "serverthread.h"
#include <QtCore>
#include <QTextStream>

#define DEBUG_SERVER 0

const char ServerThread::hi[] = "hi";
const char ServerThread::init[] = "i\n";
const char ServerThread::params[] = "p";
const char ServerThread::stop[] = "s\n";
const char ServerThread::quit[] = "q\n";

/**
 * If send_flag = 1 send data from msg
 */
void ServerThread::server_send()
{
  if (send_flag == 1) {
     // qDebug() << "Sleeping ";
     // sleep(1);
    qDebug() << "Sending message: " << server_message;
    if (send(new_socket, server_message, strlen(server_message), 0) < 0) {
      qDebug() << "Send failed";
      qDebug() << "Quitting";
      loop_flag = 0;
    }
    send_flag = 0;
  }
}

/**
 * Initialize variables
 */
void ServerThread::server_init()
{
  // initialize variables
  rx_delta_left = 0;
  rx_delta_right = 0;
  rx_omega_left = 0;
  rx_omega_right = 0;

  tx_delta_left = DEFAULT_DELTA_RAD;
  tx_delta_right = DEFAULT_DELTA_RAD;
  tx_omega_left = DEFAULT_OMEGA_RAD;
  tx_omega_right = DEFAULT_OMEGA_RAD;

  stop_server = false;
  read_size = 0;
  send_flag = 0;
  loop_flag = 0;

  memset(client_message, 0, sizeof(client_message));
  memset(server_message, 0, sizeof(server_message));
}

/**
 * Wait for incoming connection
 */
void ServerThread::server_connect()
{
  // try to connect
  //Create socket
  socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_desc == -1) {
    qDebug() << "Could not create socket";
  }

  //Prepare the sockaddr_in structure
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(SERVER_PORT);

  // set to non-blocking
  int flags;
  flags = fcntl(socket_desc, F_GETFL, 0);
  assert(flags != -1);
  //fcntl(socket_desc, F_SETFL, flags | O_NONBLOCK);

  // check for non-blocking
  flags = fcntl(socket_desc, F_GETFL, 0);
  if ((flags & O_NONBLOCK) == O_NONBLOCK) {
    qDebug() << "it's nonblocking";
  } else {
    qDebug() << "it's blocking.";
  }

  //Bind first
  if (bind(socket_desc, (struct sockaddr *) &server, sizeof(server)) < 0) {
    qDebug() << "bind failed";
    return;
  }
  qDebug() << "bind done";


  //Listen
  listen(socket_desc, 3);
  qDebug() << "Waiting for incoming connections...";

  c = sizeof(struct sockaddr_in);

  // wait for connection
  new_socket = accept(socket_desc, (struct sockaddr *) &client,
      (socklen_t*) &c);

  // 1 Sec Timeout
  tv.tv_sec  = 1;
  tv.tv_usec = 0;
  setsockopt( new_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (new_socket != -1) {
    loop_flag = 1;  // set TRUE for the main loop
    qDebug() << "Connection accepted";
  } else {
    qDebug() << "Connection error";
  }
}

/**
 * Main loop of the thread
 */
void ServerThread::run()
{
  // initialize variables first
  server_init();

  // wait for connection
  server_connect();

  // Main loop
  while (loop_flag) {
    // check new messages from client
    // read client data
    if ((read_size = recv(new_socket, client_message, SERVER_MSG_SIZE, 0))
        > 0) {
      //process
      qDebug() << "Message received: " << client_message;
      // state machine to process messages from client
      if (check_for_hi() == 1){
        // wasn't hi
        // check for init
        if (check_for_init() == 1){
          // wasn't init
          // check for stop
          if (check_for_stop() == 1) {
            // wasn't stop
            // check for params
            if (check_for_params() == 1) {
              // wasn't params
              // check for quit
              if (check_for_quit() == 1) {
                // it wasn't quit -> bad command
                printf("Unknown command.\n");
              }
              else {
                //it was quit -> so quit
                loop_flag = 0;
              } // quit
            } // params
          } // stop
        } // init
      } // hi
    } // read
    else {
      qDebug() << "No message";
    }


    // if something changed, send new message
    switch (status) {
      case SEND_PARAMS:
        // send new parameters
        sprintf(server_message, "%s;%f;%f;%f;%f;\n", params, tx_delta_left,
            tx_delta_right, tx_omega_left, tx_omega_right);
        send_flag = 1;
        break;
      case SEND_INIT:
        // send init command
        sprintf(server_message, "%s;\n", init);
        send_flag = 1;
        break;
      case SEND_STOP:
        // send stop command
        sprintf(server_message, "%s;\n", stop);
        send_flag = 1;
        break;
      case SEND_QUIT:
        // send quit command
        sprintf(server_message, "%s;\n", quit);
        send_flag = 1;
        break;
      default:
        // by default change to idle
        status = IDLE;
        // send hi command
        sprintf(server_message, "%s;\n", "hi");
        send_flag = 1;  // just to be sure
        break;
    }  // status switch
    // send messages if any
    server_send();
  }  // main loop

  qDebug() << "OUt of WHILE";
  return;
}

/**
 * Check for HI message
 */
int ServerThread::check_for_hi() {
  if(client_message[0] == 'h'){
    qDebug() << "It was HI!";
    status = SEND_PARAMS;
    return 0;
  }
  else {
    //qDebug() << "It was NOT HI.";
    return 1;
  }
}

/**
 * Check for INIT message
 */
int ServerThread::check_for_init(){
  if(client_message[0] == 'i'){
    qDebug() << "It was INIT done!";
    status = SEND_PARAMS;
    return 0;
  }
  else {
    //qDebug() << "It was NOT init_done.";
    return 1;
  }
}

/**
 * Check for stop message
 */
int ServerThread::check_for_stop() {
  if(client_message[0] == 's'){
    qDebug() << "It was STOP!";
    return 0;
  }
  else {
    //qDebug() << "It was NOT STOP.";
    return 1;
  }
}

/**
 * Check for Params message
 */
int ServerThread::check_for_params() {
  if(client_message[0] == 'p'){
      if (DEBUG_SERVER) {
    qDebug() << "It was PARAMS!";
    qDebug() << "Parsing this message: " << &client_message[2];
        }
    int parsed;
    parsed = sscanf(&client_message[2], "%f;%f;%f;%f", &rx_delta_left, &rx_delta_right, &rx_omega_left, &rx_omega_right);
    if (DEBUG_SERVER) {
        qDebug() << "Parsed = " << parsed;
      }
    else {
        (void)parsed;
      }
    return 0;
  }
  else {
    //qDebug() << "It was NOT PARAMS.";
    return 1;
  }
}

/**
 * Check for quit message
 */
int ServerThread::check_for_quit() {
  if(client_message[0] == 'h'){
    qDebug() << "It was QUIT!";
    return 0;
  }
  else {
    //qDebug() << "It was NOT QUIT.";
    return 1;
  }
}
