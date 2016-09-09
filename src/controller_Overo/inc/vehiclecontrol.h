#ifndef VEHICLECONTROL_H

#define VEHICLECONTROL_H

// Author - Hermanus Botha - Wright State University - November 15 2014

//#define PI 3.141592653589793 


/*
    All comments are in .c file above each function.
    
    DO NOT SEND VALUES FOR OMEGA GREATER THAN 24*PI THIS IS THE 
    PHYSICAL LIMIT FOR THE VEHICLE THAT I HAVE TESTED, ANY FASTER
    MAY BREAK LINKAGES AND WHATEVER YOU CAN IMAGINE ON THE VEHICLE.
    IF YOU WANT TO USE VALUES HIGHER THAN 24*PI, THEN YOU WILL HAVE TO
    CHANGE THE SOURCE CODE AND DO SO.
*/

double get_Delta(int wing, int ActPas);
double get_Omega(int wing, int ActPas);

// Call this, for each wing including a frequency.
// it will maintain the relative delta along with frequency changes.
int set_freq(int wingside, double nOmega);

int set_freq_b(int wingside, double nOmega, int ActPas);

// Call this for each wing including a delta.
// it will change the delta. 
// Delta CANNOT be larger than Omega, it will not send
// IF DELTA IS TOO SMALL, THEN WINGS/MOTORS WILL DO STRANGE THINGS
// THEN VEHICLE SHOULD BE STOPPED MANUALLY. THIS VALUE CHANGES
// DEPENDING ON VEHICLE BUILD. PLEASE TEST TO SEE WHAT THE VALUE IS
// AND MAKE SURE NOT TO SEND THAT RELATIVE DELTA VALUE.
int set_delta(int wingside, double delta);

int set_delta_b(int wingside, double delta, int ActPas);

// Call this once, at the beginning before doing anything.
// add a sleep just to make sure that the initialization completes
// a sleep(5) should be enough.
int init_wings();

int start_wing();

// IF THIS IS CALLED AT THE END OF AN EXPERIMENT, THEN THE PIC'S AND
// CONTROLL BOARD WILL HAVE TO BE RESET. UNTIL NEW FIRMWARE IS LOADED
// DO NOT CALL THIS UNLESS YOU PLAN ON RESETTING THE PIC'S MANUALLY.
int deinit_wings();

int set_direction(int dir, int Wing);

int switch_control();

int free_memory();


// not needed in this version yet.

//int send_commutation(int wingside, double* commutation);

//int en_dis(int COMMAND);

//int switch_seq();

//int freq_seq();



//double Omega = 12.56; // 2hz or 4PI

//double Delta = 6.28; // 1hz or 2PI 

	      

#endif
