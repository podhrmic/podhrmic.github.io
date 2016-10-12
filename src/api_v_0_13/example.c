#include<stdlib.h>
#include<stdio.h>
#include<string.h> 
#include"vehiclecontrol.h"

// Author - Hermanus Botha - Wright State University - November 15 2014

//#define PI 3.14159265358979323846264338327950288419


int one_rot(){
    // run vehicle with a 1/4 cycle split cosine at 1rotation/sec
    //wing 1
    int r = 1;
    r = set_freq(0, 12.56); // 2PI (1 rotation per second - theoretically)
    if(r == -1) return r;
    r = set_delta(0, 3.14); // 0.5PI (split cycle at 1/4 of a rotation)
    return r;
    
    //wing 2
    //set_freq(1, 6.28); // 2PI (1 rotation per second - theoretically)
    //set_delta(1, 1.57); // 0.5PI (split cycle at 1/4 of a rotation)
    
    //return 0;
}

int two_rot(){
    // run vehicle with a 1/4 cycle split cosine at 2rotation/sec
    //wing 1
    int r = 1;
    r = set_freq(0, 24.00); // 4PI (2 rotation per second - theoretically)
    if(r == -1) return r;
    return r;
    
    //wing 2
    //set_freq(1, 6.28); // 4PI (2 rotation per second - theoretically)
    //set_delta(1, 1.57); // PI (split cycle at 1/4 of a rotation)
    
    return 0;
}

int wing_zero_stop(){
    set_freq(0, 0); // 0 freq for wing 0
    
    return 0;
}

// This is an example set of code, this is not pretty nor is it something
// that provides a startup. This is simply here to show examples of how
// to command frequency and delta in a very basic way.
// This code is not bug free. The library in use is also a work in progress
// the basic functionality to command freq and delta on a per wing basis
// is available. More work and features are being added over time.
 
int main(int argc , char *argv[])
{
    init_wings(); // MUST BE DONE AT LEAST ONCE AFTER A RESET DEVICE
    sleep(1); // sleep to let the slow start procedure finish
    int r = 1;
    
    while(1){
        // UPON FIRST FREQUENCY COMMAND, YOU MUST ALSO SPECIFY A DELTA VALUE.
        // THE DEFAULT DELTA IS 1/2 CYCLE, THEREFORE NOT SPLIT CYCLING
        // ONLY AFTER SPECIFYING A DELTA SUCH AS 1/4 CYCLE, CAN A SPLIT CYCLE
        // BE GENERATED
    
        while(r != 0) r = one_rot(); // one rotation with 1/4 cycle split cosine delta
        r = 1;
        
        sleep(5);
        
        while(r != 0) r = two_rot(); // two rotations with 1/4 cycle split cosine
        r = 1;
        
        sleep(5);

        set_freq(0,0);

        sleep (3);
        
        //inc_freq(25.13); // increase frequency to 8PI, delta automatically scales
                         // with omega so it will be 1/4 cycle rotation in this case
                         // or 6.28 (2PI)
        
        //sleep(5);
        
        //change_delta(6.00); // since delta is 6.28 (2PI) here, we can simply shift 
                            // it down a little, if needed.
                            
        //sleep(5);
        
        //wing_zero_stop();
        // stopping a wing will cause the defaulting delta to become 1/2 cycle cosine
        // which means that when the wing is given a frequency again, it will have
        // a delta cosine which is not 1/4 cycle split cosine. YOU MUST COMMAND
        // THE WING "WAKING" UP WITH A DELTA VALUE THAT YOU WISH TO HAVE
        
        //sleep(5);
    }
    
     
    return 0;
}