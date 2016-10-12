Hermanus Botha - Wright State University - November 15 2014

MAKEFILE COMMANDS
TO COMPILE THE LIBRARY, move to the /src dir and execute the following command:
$ make

This will compile and produce a libvehicle.a library dir and a header file in 
the include directory and also a object file in the object directory which are 
all located one dir above the /src dir.

TO CLEAN THE SOURCE/OBJECT/LIBRARY execute the following command:
$ make clean

This will delete the contents of /inc, /lib, and /obj. Then you can recompile
again

TO CLEAN THE ENTIRE SOURCE INCLUDING DIRECTORIES, execute the following command:
$ make cleanall

This will delete /inc, /lib, /obj.

**--------------------------------------------------------------------------**

COMPILING SOURCE CODE THAT MAKES USE OF THE LIBRARY:
Make sure that you are in the directory where your code is located. Now execute 
the following command:
$ gcc -I <INCLUDE> -L <LIBRARY> <SOURCE>.c <LIBRARY>/libvehicle.a

Making sure to direct the directories to the locations where the library was 
built to. OR directories where the header and library are located. This is a 
simple gcc compile command, which you can alter to whatever build tool/compiler 
that you desire. 

There is a supplimentary Makefile above the /src directory that you can modify
if needed.

**--------------------------------------------------------------------------**

EXAMPLE CODE
There is sample source code included named example.c which contains very simple
sample code which you can use to familiarize yourself with the functions that
allow you to command each wing of the vehicle with frequency and delta. The 
documentation for the functions are included in the source code located in the 
/src directory. The documentation is not complete nor is it perfect, and I 
am more than willing to clarify and adjust the comments and documentation if 
needed.

The sample code simply performs a wing/motor initialization phase, which takes
up to 20 seconds. This needs only to be run ONCE AFTER A VEHICLE IS STARTED 
FOR THE FIRST TIME OR AFTER BEING RESET.
The init_wings() function MUST BE EXECUTED AT LEAST ONCE BEFORE ANY WING 
COMMANDS CAN BE MADE. THE FUNCTION MUST BE EXECUTED AT LEAST ONCE AFTER A 
VEHICLE RESET IS PERFORMED. It can be used multiple times during a session but 
is not recommended. 

The deinit_wings() should NOT be used UNLESS THE USER IS WILLING TO MANUALLY 
RESET THE VEHICLE AFTER CALLING THE FUNCTION. In the future, I will release some
PIC microcontroller firmware updates. Which must be manually applied. The 
function will THEN be able to be used safely. 

**--------------------------------------------------------------------------**

FREQUENCY:
set_freq(int wingside, double nOmega)

To command flapping frequency, a user can call the set_freq() function, which 
requires 2 parameters.
1) A value of 0 OR 1, which specifies one wing or the other.
2) A double value which specifies the Omega parameter which is the frequency 
parameter measured in rad/sec.

The function will command a single wing to move at the frequency provided. IT IS
VERY IMPORTANT TO NOTE THAT THE FLAPPING FREQUENCY OF THIS VEHICLE IS LIMITED 
BY IT'S PHYSICAL CHARACTERISTICS! Linkages and parts can break easily if the 
vehicle is pushed beyond what it can physically handle. YES THIS VEHICLE IS 
CONSTRAINED BY OUR PHYSICAL WORLD AND IT'S PHYSICAL LAWS. So the user must take 
caution when commanding the vehicle to dangerous frequencies. Currently, I 
recommend not flapping the vehicle over 24PI. If the user wishes to go beyond 
this limit, the user can adjust the source code in /src to allow for higher 
frequencies.

**--------------------------------------------------------------------------**

DELTA:
set_delta(int wingside, double delta)

To command delta frequency, a user can call the set_delta() function, which 
requires 2 parameters.
1) A value of 0 OR 1, which specifies one wing or the other.
2) A double value which specifies the Delta parameter which is the parameter 
that defines the split cycle dividing position measured in rad/sec.

This function will adjust the delta parameter associated with the current 
flapping frequency. This parameter is responsible for generating a split-cycle 
cosine. THIS VALUE CANNOT BE LARGER THAN OMEGA AND WILL NOT BE APPLIED IF LARGER
THAN OMEGA. 