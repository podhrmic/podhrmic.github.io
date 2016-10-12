#include<stdlib.h>
#include<stdio.h>
#include<string.h> 
extern "C"
{
#include"vehiclecontrol.h"
}
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include <unistd.h>
#include <iostream>


#define PI 3.141592

using namespace std;

#define LEFT 1
#define RIGHT 0

int delta_left;
int delta_right;
int omega_left;
int omega_right;


float delta_left_rad;
float delta_right_rad;
float omega_left_rad;
float omega_right_rad;

void stop_wings(void) {
	set_delta(LEFT, 0);
	set_delta(RIGHT, 0);
	set_freq(LEFT, 0); // change freq
	set_freq(RIGHT, 0); // change freq
	deinit_wings();
}

// This is an example set of code, this is not pretty nor is it something
// that provides a startup. This is simply here to show examples of how
// to command frequency and delta in a very basic way.
// This code is not bug free. The library in use is also a work in progress
// the basic functionality to command freq and delta on a per wing basis
// is available. More work and features are being added over time.

int main(int argc , char *argv[])
{
	delta_left = 0;
	delta_right = 0;
	omega_left = 0;
	omega_right = 0;

	delta_left_rad = 0;
	delta_right_rad = 0;
	omega_left_rad = 0;
	omega_right_rad = 0;

	int cnt = 0;

	int socket_desc;
	struct sockaddr_in server;
	char message[] = "00.000; 00.000; 00.000; 00.000";
	char server_reply[2000] = ""; // message size can be adjusted

	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0); // I can make it a TCP or UDP still
	if (socket_desc == -1)
	{
		printf("Could not create socket");
	}

	server.sin_addr.s_addr = inet_addr("192.168.2.10"); //Camera machine
	server.sin_family = AF_INET;
	server.sin_port = htons( 25565 ); // can pick any port

	int read_size;

	//Connect to remote server
	if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
	{
		puts("connect error");
		return 1;
	}
	puts("Connected");

	// init vehicle
	printf("Init vehicle\r\n");
	init_wings(); // MUST BE DONE AT LEAST ONCE AFTER A RESET DEVICE
	sleep(3);
	printf("Init finished\r\n");

	while(1){
		if( send(socket_desc, message, strlen(message), 0) < 0)
		{
			puts("Send failed");
			stop_wings();
			return 1;
		}

		if( recv(socket_desc, server_reply , 2000 , 0) > 0) // data from server
		{
			//printf("Reply received: %s\n", server_reply);


			// If I am closing connetion
			if(strcmp(&server_reply[0], "q") == 0){
				sprintf(message, "%i,%i;%i,%i;\n", 0, 0, 0, 0);
				send(socket_desc , message , strlen(message), 0);
				shutdown(socket_desc, 2);
				stop_wings();
				printf("quit\n");
				break;
			}
			else {
				// Parse message
				sscanf(server_reply, "%f,%f;%f,%f;\n", &delta_left_rad, &delta_right_rad, &omega_left_rad, &omega_right_rad);

				if ((delta_left_rad==0) && (delta_right_rad=0) && (omega_left_rad==0) && (omega_right_rad==0)) {
					stop_wings();
					printf("quit\n");
					break;
				}

				cnt++;
				cout << "Round: " << cnt << endl;

				// Left frequency
				cout << "RX O_L: " << omega_left_rad << endl;
				set_freq(LEFT, omega_left_rad);
				omega_left_rad = get_Omega(LEFT);
				cout << "TX O_L: " << omega_left_rad << endl;

				// Right frequency
				cout << "RX O_R: " << omega_right_rad << endl;
				set_freq(RIGHT, omega_right_rad);
				omega_right_rad = get_Omega(RIGHT);
				cout << "TX O_R: " << omega_right_rad << endl;

				// Left delta
				cout << "RX D_L: " << delta_left_rad << endl;
				set_delta(LEFT, (double)delta_left_rad);
				delta_left_rad = get_Delta(LEFT);
				cout << "TX D_L: " << delta_left_rad << endl;

				// Right delta
				cout << "RX D_R: " << delta_right_rad << endl;
				set_delta(RIGHT, (double)delta_right_rad);
				delta_right_rad = get_Delta(RIGHT);
				cout << "TX D_R: " << delta_right_rad << endl;

				cout << "\n" << endl;

				// put data back into the message
				sprintf(message, "%f,%f;%f,%f;\n", delta_left_rad, delta_right_rad, omega_left_rad, omega_right_rad);
				//send(socket_desc , message , strlen(message), 0);
			}

			memset(server_reply, 0, 2000);
		} else if(read_size < 0) {
			puts("recv failed");
			stop_wings();
		}
	}

	// Deinit vehicle
	puts("Quitting..");
	stop_wings();
	return 0;
}
