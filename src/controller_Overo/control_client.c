/*
 * Author - Hermanus Botha - Wright State University - July 4 2015
 * Modified by Michal Podhradsky, Portland State University, podhrad@pdx.edu, October 2015
 *
 * Basic communication for Gumstix Overo
 */
#include "control_client.h"

struct Vehicle veh;

static const char hi[] = "hi";
static const char init[] = "i\n";
static const char stop[] = "s\n";
static const char quit[] = "q\n";

/**
 * Send hi
 */
int send_hi(void){
  sprintf(veh.message, "%s", hi);
  // send proper response
  send(veh.socket_desc, veh.message, strlen(veh.message), 0);
  return 0;
}

/**
 * Checks for "param" command
 * return 0 if the command was "param"
 */
int check_for_params(void){
  if(veh.server_reply[0] == 'p'){
    if (veh.state == VEH_INIT){
      // set params
      printf("**************************************************\n");
      printf("PARAMETERS RECEIVED\n");
      printf("SERVER REPLY: %s;;;\n",veh.server_reply);
      int parsed;
      parsed = sscanf(&veh.server_reply[2], "%f;%f;%f;%f", &veh.rx_delta_left_rad, &veh.rx_delta_right_rad, &veh.rx_omega_left_rad, &veh.rx_omega_right_rad);
      printf("Parsed numbers = %d\n", parsed);
      printf("PARSED VALUES: %f;%f;%f;%f;\n", veh.rx_delta_left_rad, veh.rx_delta_right_rad, veh.rx_omega_left_rad, veh.rx_omega_right_rad);
      printf("**************************************************\n");


      // check for values
      //if ((veh.delta_left_rad==0) || (veh.delta_right_rad=0) || (veh.omega_left_rad==0) || (veh.omega_right_rad==0)) {
      //    printf("Received zero value for parameters.\n");
      //}
      //else {
        // values OK
        // Left frequency
        printf("%s%f\n", "RX O_L: ", veh.rx_omega_left_rad);
        printf("SETTING OMEGA LEFT PARAMETER\n");
#if !DEBUG
        set_freq_b(LEFT, veh.rx_omega_left_rad, 0);
        set_freq_b(LEFT, veh.rx_omega_left_rad, 1);
        veh.omega_left_rad = get_Omega(LEFT, 0);
#else
        veh.omega_left_rad = veh.rx_omega_left_rad;
#endif
        printf("%s%f\n", "TX O_L: ", veh.omega_left_rad);

        // Right frequency
        printf("%s%f\n", "RX O_R: ", veh.rx_omega_right_rad);
        printf("SETTING OMEGA right PARAMETER\n");
#if !DEBUG
        set_freq_b(RIGHT, veh.rx_omega_right_rad, 0);
        set_freq_b(RIGHT, veh.rx_omega_right_rad, 1);
        veh.omega_right_rad = get_Omega(RIGHT, 0);
#else
        veh.omega_right_rad = veh.rx_omega_right_rad;
#endif
        printf("%s%f\n", "TX O_R: ", veh.omega_right_rad);

        // Left delta
        printf("%s%f\n", "RX D_L: ", veh.rx_delta_left_rad);
        printf("SETTING DELTA LEFT PARAMETER\n");
#if !DEBUG
        set_delta_b(LEFT, veh.rx_delta_left_rad, 0);
        set_delta_b(LEFT, veh.rx_delta_left_rad, 1);
        veh.delta_left_rad = get_Delta(LEFT, 0);
#else
        veh.delta_left_rad = veh.rx_delta_left_rad;
#endif
        printf("%s%f\n", "TX D_L: ", veh.delta_left_rad);

        // Right delta
        printf("%s%f\n", "RX D_R: ", veh.rx_delta_right_rad);
        printf("SETTING DELTA RIGHT PARAMETER\n");
#if !DEBUG
        set_delta_b(RIGHT, (double)veh.rx_delta_right_rad, 0);
        set_delta_b(RIGHT, (double)veh.rx_delta_right_rad, 1);
        veh.delta_right_rad = get_Delta(RIGHT, 0);
#else
        veh.delta_right_rad = veh.rx_delta_right_rad;
#endif
        printf("%s%f\n", "TX D_R: ", veh.delta_right_rad);
      //}

    }
    else {
      // vehicle not running
      printf("**************************************************\n");
      printf("Not ready (stopped or not initialized.\n");
      printf("**************************************************\n");
    }

    // prepare response
    sprintf(veh.message, "p;%f;%f;%f;%f;\n", veh.delta_left_rad, veh.delta_right_rad, veh.omega_left_rad, veh.omega_right_rad);

    // command found
    return 0;
  }
  else {
    // continue search
    return 1;
  }
}

/**
 * Checks for "quit" command
 * return 0 if the command was "quit"
 */
int check_for_quit(void){
  if(veh.server_reply[0] == 'q'){
      printf("**************************************************\n");
      printf("I AM CLOSING CONNECTION\n");
      printf("**************************************************\n\n");
      veh.state = VEH_STOP;

      // prepare response
      sprintf(veh.message, "%s", quit);

      // command found
      return 0;
  }
  else {
    // continue search
    return 1;
  }
}

/**
 * Checks for "stop" command
 * return 0 if the command was "stop"
 */
int check_for_stop(void){
  if(veh.server_reply[0] == 's'){
    if (veh.state == VEH_INIT) {
        printf("**************************************************\n");
        printf("STOPPING WINGS\n");
#if !DEBUG
        stop_wings();
#endif
        printf("WINGS HAVE BEEN STOPPED\n");
        printf("**************************************************\n\n");
        veh.state = VEH_UNINIT;
    }
    else {
      printf("**************************************************\n");
      printf("Already stopped.\n");
      printf("**************************************************\n");
    }
    // prepare response
    sprintf(veh.message, "%s", stop);

    // command found
    return 0;
  }
  else {
    // continue search
    return 1;
  }
}

/**
 * Checks for init
 * return 0 if the command was "init"
 */
int check_for_init(void){
  if(veh.server_reply[0] == 'i'){
    if (veh.state == VEH_UNINIT) {
      // init vehicle
      printf("**************************************************\n");
      printf("I AM INITIALIZING THE VEHICLE\n");
      printf("I WILL INITIALIZE IN 2 STAGES\n");
      printf("THIS CAN TAKE UP TO 30 SECONDS\n");
#if !DEBUG
      init_wings(); // MUST BE DONE AT LEAST ONCE AFTER A RESET DEVICE
#endif
      printf("PREPARING TO A CONTROL SWITCH ROUTINE\n");
#if !DEBUG
      sleep(3);
      switch_control();
#endif
      printf("PREPARING TO RETURN CONTROL SWITCH ROUTINE\n");
#if !DEBUG
      sleep(3);
      switch_control();
#endif
      printf("SLOW START INITIALIZATION COMPLETED\n");
      printf("**************************************************\n\n");
      veh.state = VEH_INIT;
    }
    else {
      printf("**************************************************\n");
      printf("Already initialized.\n");
      printf("**************************************************\n");
    }
    // prepare response
    sprintf(veh.message, "%s", init);

    // command found
    return 0;
  }
  else {
    // continue search
    return 1;
  }
}

/**
 * Set wings to zero frequency
 */
void stop_wings(void){
#if !DEBUG
  set_freq_b(0, 0, 0);
  set_freq_b(1, 0, 0);
#endif
}

/**
 * Initialize the program
 */
void init_vehicle(void){
  // vehicle state
  veh.state = VEH_UNINIT;
  veh.new_cmd = 0; // no new data

  // default parameters
  veh.delta_left_rad = DEFAULT_DELTA_RAD;
  veh.delta_right_rad = DEFAULT_DELTA_RAD;
  veh.omega_left_rad = DEFAULT_OMEGA_RAD;
  veh.omega_right_rad = DEFAULT_OMEGA_RAD;

  veh.rx_delta_left_rad = DEFAULT_DELTA_RAD;
  veh.rx_delta_right_rad = DEFAULT_DELTA_RAD;
  veh.rx_omega_left_rad = DEFAULT_OMEGA_RAD;
  veh.rx_omega_right_rad = DEFAULT_OMEGA_RAD;

  // com variables
  veh.cnt = 0;
  veh.flag = 1;
  memset(veh.message, 0, sizeof(veh.message));
  memset(veh.server_reply, 0, sizeof(veh.server_reply));
}

/**
 * Connect to servre
 */
int connect_server(void){
  //Create socket
  veh.socket_desc = socket(AF_INET , SOCK_STREAM , 0);
  if (veh.socket_desc == -1)
  {
    printf("Could not create socket");
    return 1;
  }

  // controlling server machine
  veh.server.sin_addr.s_addr = inet_addr(SERVER_ADDR);
  veh.server.sin_family = AF_INET;
  veh.server.sin_port = htons( SERVER_PORT ); // can pick any port

  //Connect to remote server
  if (connect(veh.socket_desc, (struct sockaddr *)&veh.server, sizeof(veh.server)) == 0)
  {
    // Return 0 on success
    puts("Connected");
    return 0;
  }
  else {
    // Error
    puts("Connection error.");
    return 1;

  }
}

int main(int argc , char *argv[])
{
  // init vehicle
  init_vehicle();

  // connect to server
  if(connect_server() != 0) {
    // error
    puts("Closing.");
    return 0;
  }

  // connected
  puts("All peachy.");
  puts("Sending hi.");


  // lets go in big loop and process incoming data
  while(veh.flag){
    // Sort of hearbeat
    //send_hi();

    // recv is blocking call
    // data from server
    veh.cnt = recv(veh.socket_desc, veh.server_reply , SERVER_MSG_SIZE , 0);
    if (veh.cnt > 0 ) {
      // we received data
      // print to terminal that a message from server came in
      printf("RECEIVED: %d BYTES\n", veh.cnt);
      printf("MSG FROM SERVER: %s\n", veh.server_reply);

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
              veh.flag = 0;
            }
          }
        }
      }

      // send proper response
      send(veh.socket_desc, veh.message, strlen(veh.message), 0);
    }
    else {
      puts("Received no data.");
      // no data -> server probably ended the connection
      veh.flag = 0;
    }

    // erase server reply
    memset(veh.server_reply, 0, SERVER_MSG_SIZE);
    // erase message
    memset(veh.message, 0, SERVER_MSG_SIZE);
    // erase counter
    veh.cnt = 0;
    // sleep 500ms
    //nanosleep((const struct timespec[]){{0, NSLEEP_TIME}}, NULL);

  } // veh.flag

  puts("Closing program.");
  shutdown(veh.socket_desc, 2); // close socket
  if (veh.state != VEH_UNINIT){
    // dont call unless properly initialized
    stop_wings();
  }
  return 0;
}
