#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <vehiclecontrol.h>
#include <inttypes.h>
#include <string.h>

// Author - Hermanus Botha - Wright State University - November 15 2014

// Sample code used and adjusted from:
// http://bdml.stanford.edu/Profiles/UsingTheGumstixCOM
// http://www.gumstix.org/software-development

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

#define TABLESIZE 1024

//array size macro ONLY IF THE SCOPE HASN'T BEEN LOST. SUCH AS A FUNCTION ARGUMENT (then it's a pointer, and then the macro fails)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
//                     Controller 0 ,  Controller 1
//                       W0 ,   W1       W0 ,   W1
double Omega[2][2] = {{12.56, 12.56}, {12.56, 12.56}}; // default omega value
double Delta[2][2] = {{6.28, 6.28}, {6.28, 6.28}}; // default delta value
uint16_t Direction = 1; // 1 = cw ; 0 = ccw

uint32_t *Table0;
uint32_t *Table1;

static void pabort(const char *s)
{
    perror(s);
    abort();
}

int spi1d, spi2d; //global spi devices 2d = right side (when motors are on lower side). 1d = left side (when motors are on lower side).
// global motor control values
int controlA = 0; int controlB = 0;
int disA = 0; int disB = 0;
// for each wing, there is an active controller
int activContr[2] = {0, 0};

//SPI devices declaration
static const char *device1 = "/dev/spidev1.0"; //left side (when motors are on lower side)
static const char *device2 = "/dev/spidev3.0"; //right side (when motors are on lower side)
static uint8_t mode = SPI_MODE_3;
static uint8_t bits = 16;
static uint32_t speed = 8000000; // 8MHz
static uint32_t delay;

// struct timespec time;
// time.tv_sec = 0;
// time.tv_nsec = 1000000L; // 1millisec

const struct timespec ti = {.tv_sec = 0, .tv_nsec = 1000000L}; 

/*
    This is going to be a function that takes a 16bit value builds a packet
    and transfers it to the selected spi interface
*/
static uint16_t SPItx16b(int spid, uint16_t data)
{
    int ret;
    //just default arrays
    uint16_t tx[1];
    uint16_t rx[ARRAY_SIZE(tx)];
    
    tx[0] = data;

    struct spi_ioc_transfer tr =
    {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = sizeof(tx), //size of entire message in bytes. NOT the array size
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    ret = ioctl(spid, SPI_IOC_MESSAGE(1), &tr);
    //ret = write(spid, &tx, 16);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to transmit SPI message\n");
        // maybe kill program????
    }
    //  }
    //read(spid, &rx, 16);
        // for (ret = 0; ret < ARRAY_SIZE(rx) ; ret++)
        // {
        //     printf("returned: 0x%.4X \n", rx[ret]);
        // } 
    //puts("");
    return (rx[0]); //spid
}

/*
    //Change pin to HIGH or LOW
    This fn uses linux userspace to write a logical high or low to the 
    selected gpio pin. 
    
    added fsync() to assure correct data transfer via userspace
*/
static int GPIOwrite(int pin, int value)
{
    static const char s_values_str[] = "01";
#define VALUE_MAX 30
    char path[VALUE_MAX];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd)
    {
        fprintf(stderr, "Failed to open gpio value for writing!\n");
        return (-1);
    }
    if (write(fd, &s_values_str[LOW == value ? 0 : 1], 1) < 0)   // previously != 1
    {
        fprintf(stderr, "Failed to write value!\n");
        return (-1);
    }
    fsync(fd);
    close(fd);
    return (0);
}

/*
    Open pin up for userspace access
    
    This fn will allow a GPIO pin to become available for use via userspace
    linux access. This is currently prefered as it is less complex than 
    direct register access via kernel space.
    
    added sprintf() as some errors were noticed as well as fsync() to assure
    correct data transfer via userspace.
*/
static int GPIOexport(int pin)
{
#define BUFFER_MAX 3 
    char buffer[BUFFER_MAX]; //overcompensating
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY); // there is a flag O_DIRECT which bypasses cache. Possibly look into that
    if (fd == -1)
    {
        fprintf(stderr, "Failed to open export for writing!\n");
        return (-1);
    }
    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    sprintf(buffer, "%d", pin); //Without this, the export doesn't work!?!?!?
    write(fd, buffer, bytes_written);
    fsync(fd);
    close(fd);
    return (0);
}

static int GPIOunexport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd)
    {
        fprintf(stderr, "Failed to open unexport for writing!\n");
        return (-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    sprintf(buffer, "%d", pin);
    write(fd, buffer, bytes_written);
    fsync(fd);
    close(fd);
    return (0);
}

/*
    Set pin direction to OUTPUT or INPUT
    Pin can't be more than 3 chars wide, dir IN=0, OUT=1
    
    This will take a GPIO pin and force it to become either an input pin
    or and output pin. This allowing to be used as a flag. This specific 
    implementation limits the "pin" variable to only 3 characters wide.
    Also, the variable dir can only be "in" or "out"
*/
static int GPIOdirection(int pin, int dir)
{
    static const char s_directions_str[] = "in\0out";
#define DIRECTION_MAX 35 // this is for directory chars + pin chars (and 2 extra chars)
    char path[DIRECTION_MAX]; // this will gold the path
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd)
    {
        fprintf(stderr, "Failed to open gpio direction for writing!\n");
        return (-1);
    }

    if (write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3) < 0)
    {
        fprintf(stderr, "Failed to set direction!\n");
        return (-1);
    }
    fsync(fd);
    close(fd);
    return (0);
}

static int sendBroken(int spiDevice){
    uint16_t BROKEN = 1111; 
    uint16_t cmd = 0;
    while(cmd != 1111){
        cmd = SPItx16b(spiDevice, BROKEN);
    }
    return 0;
}

// static int sendGarbage(int spiDevice){
//     uint16_t GARBAGE = 5555; 
//     uint16_t cmd = 0;
//     while(cmd != 5555){
//         cmd = SPItx16b(spiDevice, GARBAGE);
//     }
//     return 0;
// }

static int findSpidev(int Wing, int TableSelect, int SpiSelect){
    if(Wing == 0){
        GPIOwrite(144, SpiSelect);  // left wing SPI_S
        GPIOwrite(145, TableSelect); // left wing TABLE_S must be inverted from SPI_S
        return spi1d;
    } else if(Wing == 1){
        GPIOwrite(146, SpiSelect); //  right wing SPI_S
        GPIOwrite(147, TableSelect); // right wing TABLE_S must be inverted from SPI_S
        return spi2d;
    }
    return (0);
}

static int transferLoop(int dev, uint16_t DATA){
    // struct timespec time;
    // time.tv_sec = 0;
    // time.tv_nsec = 1000000L; // 1millisec

    uint16_t GARBAGE = 5555; // just a garbage value for the final message
    uint16_t cmd = 0;
    int failcount = 0;
    while(cmd != DATA){
        cmd = SPItx16b(dev, DATA); // send command for 16 bit transfer, recieve cmd which will be the previous sent message. cmd here should be grabage.
        nanosleep(&ti, NULL); // wait for everything to settle
        cmd = SPItx16b(dev, GARBAGE); // send garbage to check if last transfer was success
        nanosleep(&ti, NULL); // wait for everything to settle
        if(failcount >= 5) return (-1);
        failcount++;
    }
    return (failcount);
}

static int transfer(int dev, uint16_t DATA){
    // struct timespec time;
    // time.tv_sec = 0;
    // time.tv_nsec = 1000000L; // 1millisec

    uint16_t GARBAGE = 5555; // just a garbage value for the final message
    uint16_t cmd = 0;
    cmd = SPItx16b(dev, DATA); // send command for 16 bit transfer, recieve cmd which will be the previous sent message. cmd here should be grabage.
    nanosleep(&ti, NULL); // wait for everything to settle
    cmd = SPItx16b(dev, GARBAGE); // send garbage to check if last transfer was success
    nanosleep(&ti, NULL); // wait for everything to settle
    if(cmd != DATA) return (-1);
    return (0);
}

/*
    This is a function that will recieve:
    wing - Left or Right wing (PIC): 1 or 2 respectively OR 0 or 1 respectively
    ts - Table Select (either in controll or not)
    ss - Spi Select (enable or disable spi for selected wing)
    delta 1 - first delta value 
    delta 2 - second delta value 

    This function will coordinate the send of values to the PIC

    When this function is finished, the selected wing will have 2 values
    handed over SPI and confirmed correct.

    command 11 or 00 will give control and spi to the same wing(PIC). command 10 or 01 will give command to one wing and spi to the other.
    In the case of setShortParams, the control will be on same wing. Send commands over SPI to the controlling wing.
*/
static int setShortParams(int Wing, int TableSelect, int SpiSelect, uint16_t Delta1, uint16_t Delta2, uint16_t Sign){
    //uint16_t GARBAGE = 5555; // just a garbage value for the final message
    //uint16_t BROKEN = 1111; 
    uint16_t COMMAND = 10; //will make it 10 for now
    uint16_t COMMIT = 1001;
    //uint16_t cmd = 0; // cmd will be previous sent command as recieved from slave device
    int spiDevice = -1;

    // struct timespec time;
    // time.tv_sec = 0;
    // time.tv_nsec = 1000000L; // 1millisec

    spiDevice = findSpidev(Wing, TableSelect, SpiSelect);

    if(spiDevice < 0) return(-1); // this means that the wing command that came in was a fail

    if(transferLoop(spiDevice, COMMAND) < 0) return (-1);
    if(transfer(spiDevice, Delta1) < 0) return (-1);
    if(transfer(spiDevice, Delta2) < 0) return (-1);
    if(transfer(spiDevice, Sign) < 0) return (-1);
    if(transferLoop(spiDevice, COMMIT) < 0) return (-1);

    return (0);
}

/*
    This is a function that will recieve:
    wing - Left or Right wing (PIC): 1 or 2 respectively OR 0 or 1 respectively
    ts - Table Select (either in controll or not)
    ss - Spi Select (enable or disable spi for selected wing)
    table - commutation table array 

    This function will coordinate the send values to the PIC

    When this function is finished, the selected wing will have a commutation table
    handed over SPI and confirmed correct.

    command 11 or 00 will give control and spi to the same wing(PIC). command 10 or 01 will give command to one wing and spi to the other.
    In the case of setShortParams, the control will be on same wing. Send commands over SPI to the controlling wing.

    I need to find out if I should be referencing the table instead of just passing it. 
    Also, what is the best way of finding the size of the array.
*/
static int setLongParams(int Wing, int TableSelect, int SpiSelect, uint32_t* Table){
/**
 *
 *    int32_t big;
 *  int16_t low = table[0] & 0x0000ffff; // this catches the low 16 bits of the 32 bit value
 *  int16_t high = table[0] >> 16;
 *  
 *  // to reconstruct
 *  
 *  big = (int32_t)high << 16 | low; // what about big = low + (high << 16);
 *  //pic -> big = low + (high << 16); // wonder if that works on a normal machine?
 * 
 */
//http://www.learncpp.com/cpp-tutorial/38-bitwise-operators/
//http://codereview.stackexchange.com/questions/30593/split-a-long-integer-into-eight-4-bit-values
//http://www.cprogramming.com/tutorial/bitwise_operators.html
//http://www.9wy.net/onlinebook/CPrimerPlus5/ch15lev1sec3.html
//http://www.cs.umd.edu/class/sum2003/cmsc311/Notes/BitOp/bitshift.html
//http://stackoverflow.com/questions/12791864/c-program-to-check-little-vs-big-endian

    // struct timespec time;
    // time.tv_sec = 0;
    // time.tv_nsec = 1000000L; // 1millisec

    uint16_t COMMAND = 11; //will make it 01 for now
    //uint16_t GARBAGE = 5555; // just a garbage value for the final message
    uint16_t COMMIT = 1002;
    //uint16_t cmd = 0;
    int spiDevice = -1;

    // THIS NEEDS TO BE CHANGED TO ALLOW FOR THE MULTIPLEX SWITCH FOR TABLE TRANSFER
    spiDevice = findSpidev(Wing, TableSelect, SpiSelect);

    if(spiDevice < 0) return(-1); // this means that the wing command that came in was a fail

    int failed = 0;
    // table size declared way up top. If we want dynamic arrays, then someone should figure out how to dynamically change the PIC array/controller
    int i = 0;
    for (i = 0; i < TABLESIZE; ++i) // ++i returns the incremented value of i, i++ returns the value of i before incrementing. For loop it doesn't matter
    {
        if(failed == 1) {
            i--; // this is to redo the last transmission.
            printf("%s%d\n", "Last entry failed: ", i);
            failed = 0;
        }

        transferLoop(spiDevice, COMMAND);

        //split the 32bit value in 2 16 bit values to send.
        uint16_t temp = 0;
        int j = 0;
        for(j = 0; j < 2; j++){
            temp = (uint16_t) (Table[i] >> (16 * j));
            // temp = (uint16_t) (Table[i] >> (16 * j)) & 0x0000FFFF;
            if(transfer(spiDevice, temp < 0)) failed = 1; // if the transfer failed we need to just send the next value and index and repeat this cycle.
        }
        
        if(transfer(spiDevice, (uint16_t)i) < 0) failed = 1;
        printf("%s%"PRIu32", %d\n", "Sent entry: ", Table[i], i);
    }

    if(transferLoop(spiDevice, COMMIT) < 0) return (-1);

    return(0);
}

/*
    This fn is responsible for setting up the SPI bus via userspace IOCTL
    calls. The devices that are handed to this function are global and will
    most likely stay global as passing userspace objects by reference can
    become ugly.
*/
static int SPIinit(const char *dev)
{
    int ret;
    int fd;
    fd = open(dev, O_RDWR);
    if (fd < 0)
        pabort("can't open device");

    /*
     * spi mode
     */
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
        pabort("can't set spi mode");

    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
        pabort("can't get spi mode");

    /*
    =         * bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't set bits per word");

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't get bits per word");

    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't set max speed hz");

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't get max speed hz");

    printf("spi mode: %d\n", mode);
    printf("bits per word: %d\n", bits);
    printf("max speed: %d MHz\n", speed / 1000000);

    return (fd);
}

/*
    Initiate wings, export pins in userspace and enable spi devices.
    This might be replaced by the init_wings() fn. Or utilized by it. 
    The values in this fn are hardcoded as they are the only GPIO pins 
    that the GUMSTIX can make use of in this system to communicate with
    the SPI slave PIC's
    
    The devices used in the SPIinit fn are global and will most probably
    stay global as passing userspace objects by reference can become ugly.
*/
/*
static int initWings(){
    // open pin's up for userspace access SPI and TABLE
    GPIOexport(144);
    GPIOexport(145);
    GPIOexport(146);
    GPIOexport(147);

    // set dir for the special pins
    GPIOdirection(146, OUT);
    GPIOdirection(147, OUT);

    spi1d = SPIinit(device1);
    spi2d = SPIinit(device2);

    return 0;
}
*/

/*
    Unexport pins and close spi devices. This might be replaced by the 
    deinit_wings() fn. Or utilized by it. The values in this fn are hardcoded
    as they are the only GPIO pins that the GUMSTIX can make use of in this
    system to communicate with the SPI slave PIC's
*/
/*
static int deinitWings(){
    GPIOunexport(147);
    GPIOunexport(146);
    GPIOunexport(145);
    GPIOunexport(144);

    close(spi1d);
    close(spi2d);

    return 0;
}
*/

/*
    This function changed GPIO either to active high or active low
    This is to say, that if you tell a active high GPIO to become high, then 
    it will produce a logical high.
    If you tell a active low GPIO to become high, then it will produce a 
    logical low.
*/

/*
static int GPIOactivelh(int pin, int value)
{
    static const char s_values_str[] = "01";
#define ACTV_MAX 35
    char path[ACTV_MAX];
    int fd;

    snprintf(path, ACTV_MAX, "/sys/class/gpio/gpio%d/active_low", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd)
    {
        fprintf(stderr, "Failed to open gpio activelow value for writing!\n");
        return (-1);
    }
    if (write(fd, &s_values_str[LOW == value ? 0 : 1], 1) < 0)   // previously != 1
    {
        fprintf(stderr, "Failed to write active low value!\n");
        return (-1);
    }
    fsync(fd);
    close(fd);
    return (0);
}
*/

/*
    This is where data is packaged and transfered. This will need only a 
    small amount of adjustment according to the protocol between gumstix 
    and PIC's.
    
    For instance, the array's for on and off can be removed and replaced with
    important data. 
*/
static int SPItx(int spid, int on)
{
    int ret;
    //just default arrays
    uint16_t tx[1];
    uint16_t rx[ARRAY_SIZE(tx)];

    uint16_t tx2[2048];
    uint16_t rx2[ARRAY_SIZE(tx2)];

    if (on == 1)
    {
        tx[0] = 0xAAAA;
    }
    else if (on == 0)
    {
        tx[0] = 0xBBBB;
    }

    struct spi_ioc_transfer tr =
    {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = sizeof(tx), //size of entire message in bytes. NOT the array size
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    struct spi_ioc_transfer tr2 =
    {
        .tx_buf = (unsigned long)tx2,
        .rx_buf = (unsigned long)rx2,
        .len = sizeof(tx2), //size of entire message in bytes. NOT the array size
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };



    //  while(1){ // huh?
    if (on == 5) ret = ioctl(spid, SPI_IOC_MESSAGE(1), &tr2);
    else ret = ioctl(spid, SPI_IOC_MESSAGE(1), &tr);
    //ret = write(spid, &tx, 16);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to transmit SPI message\n");
        return -1;
        // maybe kill program????
    }
    //  }
    //read(spid, &rx, 16);
    if (on == 5)
    {
        for(ret=0; ret < ARRAY_SIZE(rx2); ret++){
          printf("%d)", ret);
          printf("0x%.4X \n", rx2[ret]);
        }
    }
    else
        for (ret = 0; ret < ARRAY_SIZE(rx) ; ret++)
        {
            printf("0x%.4X \n", rx[ret]);
        }
    //puts("");
    return (0);
}

/*
    Used for debugging, no longer needed
*/
/*
static void print_usage(const char *prog)
{
    printf("Usage: %s [-AB]\n", prog);
    puts(
        "  -A --motora  Activate or Deactivate MOTOR A\n"
        "  -B --motorb  Activate or Deactivate MOTOR B\n"
    );
    exit(1);
}
*/

/*
    Definately not needed anymore. This was originally meant as a terminal
    app that would recieve commands and parse to make sure that correct 
    parameters were selected.
*/
/*
static void parse_opts(int argc, char *argv[])
{
    while (1)
    {
        static const struct option lopts[] =
        {
            {"motora",  0, 0, 'A'},
            {"motorb",  0, 0, 'B'},
            {"NULL",    0, 0, 0}
        };
        int c;

        c = getopt_long(argc, argv, "A:B:C:D", lopts, NULL);

        if (c == -1)
            break;

        switch (c)
        {
        case 'A':
            controlA = atoi(optarg);
            if (controlA == 20 || controlA == 3) disA = 0;
            else if (controlA == 21 || controlA == 1) disA = 1;
            break;
        case 'B':
            controlB = atoi(optarg);
            if (controlB == 20 || controlB == 3) disB = 0;
            else if (controlB == 21 || controlB == 1) disB = 1;
            break;
        default:
            print_usage(argv[0]);
            break;
        }

    }
}
*/

double get_Delta(int wing, int ActPas){
    return Delta[ActPas][wing];
}

double get_Omega(int wing, int ActPas){
    return Omega[ActPas][wing];
}

/*
    This is where all global initialization will happen
    Things like SPI 1 and 2 will become instantiated here as well as
    global variables and state variables. 
    Also, any GPIOs' that are required to make this implementation work
    will become initialised here.
*/
int init_wings()
{

    // struct timespec time;
    // time.tv_sec = 0;
    // time.tv_nsec = 100000000L; // 1millisec

    // open pin's up for userspace access SPI and TABLE
    GPIOexport(144);
    GPIOexport(145);
    GPIOexport(146);
    GPIOexport(147);
    nanosleep(&ti, NULL);

    // set dir for the special pins
    GPIOdirection(146, OUT);
    GPIOdirection(147, OUT);
    nanosleep(&ti, NULL);

    spi1d = SPIinit(device1);
    spi2d = SPIinit(device2);
    nanosleep(&ti, NULL);

    uint16_t READ_ENC = 102;
    GPIOwrite(144, 0);  // left wing SPI_S
    GPIOwrite(145, 0);
    GPIOwrite(146, 0); // left wing TABLE_S must be inverted from SPI_S
    GPIOwrite(147, 0);
    nanosleep(&ti, NULL);
    if(transferLoop(spi1d, READ_ENC) < 0) return (-1);
    if(transferLoop(spi2d, READ_ENC) < 0) return (-1);
    nanosleep(&ti, NULL);
    GPIOwrite(144, 1);  // left wing SPI_S
    GPIOwrite(145, 1);
    GPIOwrite(146, 1); // left wing TABLE_S must be inverted from SPI_S
    GPIOwrite(147, 1);
    nanosleep(&ti, NULL);
    if(transferLoop(spi1d, READ_ENC) < 0) return (-1);
    if(transferLoop(spi2d, READ_ENC) < 0) return (-1);
    nanosleep(&ti, NULL);
    GPIOwrite(144, 0);  // left wing SPI_S
    GPIOwrite(145, 0);
    GPIOwrite(146, 0); // left wing TABLE_S must be inverted from SPI_S
    GPIOwrite(147, 0);
    nanosleep(&ti, NULL);

    set_freq_b(0, 12.56, 0);
    set_freq_b(0, 12.56, 1);
    set_freq_b(1, 12.56, 0);
    set_freq_b(1, 12.56, 1);
    sleep(10);

    set_delta_b(0, 6.28, 0);
    set_delta_b(0, 6.28, 1);
    set_delta_b(1, 6.28, 0);
    set_delta_b(1, 6.28, 1);
    sleep(10);

    // // experiment with different startup command 0xAAAA - maybe the freq command or change the command routine in low-level
    // uint16_t INIT = 0xAAAA;
    // if(transferLoop(spi1d, INIT) < 0) return (-1);
    // if(transferLoop(spi2d, INIT) < 0) return (-1);
    // sleep(15);
    // GPIOwrite(144, 1);  // left wing SPI_S
    // GPIOwrite(145, 1);
    // GPIOwrite(146, 1); // left wing TABLE_S must be inverted from SPI_S
    // GPIOwrite(147, 1);
    // nanosleep(&ti, NULL);
    // if(transferLoop(spi1d, INIT) < 0) return (-1);
    // if(transferLoop(spi2d, INIT) < 0) return (-1);
    // sleep(15);
    // GPIOwrite(144, 0);  // left wing SPI_S
    // GPIOwrite(145, 0);
    // GPIOwrite(146, 0); // left wing TABLE_S must be inverted from SPI_S
    // GPIOwrite(147, 0);



    // SPItx(spi1d, 1); // enable slow start procedure wing1
    // SPItx(spi2d, 1); // enable slow start procedure wing2

    // sleep(15); // sleep for 10 to wait for slow start to complete

    // GPIOwrite(144, 1);  // left wing SPI_S
    // GPIOwrite(145, 1);
    // GPIOwrite(146, 1); // left wing TABLE_S must be inverted from SPI_S
    // GPIOwrite(147, 1);

    // SPItx(spi1d, 1); // enable slow start procedure wing1
    // SPItx(spi2d, 1); // enable slow start procedure wing2

    // sleep(15);

    // GPIOwrite(144, 0);  // left wing SPI_S
    // GPIOwrite(145, 0);
    // GPIOwrite(146, 0); // left wing TABLE_S must be inverted from SPI_S
    // GPIOwrite(147, 0);


    // Need to switch one wing SPI and command recording of encoder.
    // Then switch to other SPI and command recording of encoder.
    // DO THIS BEFORE FLAPPING. To zero starting position.


    return 0;
}

int start_wing(){
    GPIOwrite(144, 1);  // left wing SPI_S
    GPIOwrite(145, 1);
    GPIOwrite(146, 1); // left wing TABLE_S must be inverted from SPI_S
    GPIOwrite(147, 1);

    SPItx(spi1d, 1); // enable slow start procedure wing1
    SPItx(spi2d, 1); // enable slow start procedure wing2

    sleep(11);

    return 0;
}

/*
    All of the set_* functions will require access to the SPI transfer
    system. This means that each function transmits a message to a wing
    or both wings.
    
    Also, this level of API must know which control PIC is active and which
    is passive, so a state variable must be kept to know which PIC the 
    data will be transmitted to as to not disrupt a current wing movement.
    
    Also it should be noted that a frequency and delta parameter is small 
    enough to be sent to the active PIC without effecting the movement 
    dramatically. However further testing is required. 
*/

/*
    This will adapt the wing frequency. The caller will specify the wing
    and also the frequency that it should move at. 
    2 Parameters, wingside, and Omega. 
    wingside is 0 or 1 (selecting either one wing or the other)
    
    **MIGHT WANT TO LOOK AT MAKING ANOTHER FN THAT TAKES BOTH WINGS AND A FREQ
    OR 2 FREQ'S**
*/
// int set_freq(int wingside, double nOmega)
// {
    
//     //OMEGA = flapping frequency in rad/sec (2PI = 1hz?)
//     //DELTA = Split Cycle parameter (also rad/sec = PI = )
    
//     // I can rec delta/omega values from user and push to PIC
//     // values from user double, but values to PIC are uint16_t
//     // so multiply user values with 100 makes sending to pic much 
//     // much simpler
    
//     if(nOmega < 75.39){
        
//         if(Omega[wingside] == 0) { // check for 0 freq, this is dangerous, because see below
//             Omega[wingside] = 1; // We don't want to divide by zero, otherwise we create a hole in space and time
//             Delta[wingside] = 0.5; // We want to have Delta be half of Omega by default at a min
//         }
        
//         // how to maintain delta when freq changes?
//         // take ratio of change from omega to nomega, mult with delta 
//         double ratio = nOmega/Omega[wingside]; // ratio of change from last frequency param
//         //Omega = nOmega;
//         double nDelta = ratio*Delta[wingside]; // apply ratio to current delta for proper shift
//         //Delta = nDelta;
        
//         // adjust delta's for SPI transfer
//         uint16_t iOmega = nOmega*100;
//         uint16_t iDelta = nDelta*100;
//         uint16_t sign = 0; // orientation of delta (NEG or POS)
//         if(nDelta < 0) {
//             iDelta = nDelta*(-100);
//             sign = 1;
//         } 
//         else {
//             iDelta = nDelta*100;
//             sign = 0;
//         }
//         int TABLE_S = 0;
//         int SPI_S = 0;
        
//         int r = -1;
//         // send 'em
//         while(r < 0){
//             r = setShortParams(wingside, TABLE_S, SPI_S, iOmega, iDelta, sign);
//         }
//         if(r == 0){ // when sent success, set global Omega and Delta
//             Omega[wingside] = nOmega;
//             Delta[wingside] = nDelta;
//         }
//         return r;
//     } 
//     else {
//         printf("Can't send: Omega too large, O: %f \n", nOmega); // can also return 1 if needed
//         return 1;
//     }
//     return 1; // if this happens then wow
// }

int set_freq_b(int wingside, double nOmega, int ActPas)
{
    
    //OMEGA = flapping frequency in rad/sec (2PI = 1hz?)
    //DELTA = Split Cycle parameter (also rad/sec = PI = )
    
    // I can rec delta/omega values from user and push to PIC
    // values from user double, but values to PIC are uint16_t
    // so multiply user values with 100 makes sending to pic much 
    // much simpler
    
    if(nOmega < 75.39){
        
        if(Omega[ActPas][wingside] == 0) { // check for 0 freq, this is dangerous, because see below
            Omega[ActPas][wingside] = 1; // We don't want to divide by zero, otherwise we create a hole in space and time
            Delta[ActPas][wingside] = 0.5; // We want to have Delta be half of Omega by default at a min
        }
        
        // how to maintain delta when freq changes?
        // take ratio of change from omega to nomega, mult with delta 
        double ratio = nOmega/Omega[ActPas][wingside]; // ratio of change from last frequency param
        //Omega = nOmega;
        double nDelta = ratio*Delta[ActPas][wingside]; // apply ratio to current delta for proper shift
        //Delta = nDelta;
        
        // adjust delta's for SPI transfer
        uint16_t iOmega = nOmega*100;
        uint16_t iDelta = nDelta*100;
        uint16_t sign = 0; // orientation of delta (NEG or POS)
        if(nDelta < 0) {
            iDelta = nDelta*(-100);
            sign = 1;
        } 
        else {
            iDelta = nDelta*100;
            sign = 0;
        }
        int TABLE_S = 0;
        int SPI_S = ActPas;
        
        int r = -1;
        // send 'em
        while(r < 0){
            r = setShortParams(wingside, TABLE_S, SPI_S, iOmega, iDelta, sign);
        }
        if(r == 0){ // when sent success, set global Omega and Delta
            Omega[ActPas][wingside] = nOmega;
            Delta[ActPas][wingside] = nDelta;
        }
        return r;
    } 
    else {
        printf("Can't send: Omega too large, O: %f \n", nOmega); // can also return 1 if needed
        return 1;
    }
    return 1; // if this happens then wow
}

/*
    This will all init_wings work will be deinit'd. Simply to keep the
    system from "freaking-out". As this makes use of userspace linux 
    access, the objects and busses that are used must be free'd to make
    sure that everything is functioning properly.
*/
int deinit_wings()
{
    set_freq_b(0, 0, 0);
    set_freq_b(1, 0, 0);
    
    GPIOunexport(147);
    GPIOunexport(146);
    GPIOunexport(145);
    GPIOunexport(144);

    close(spi1d);
    close(spi2d);
    
    return 0;
}

/*
    This adapts the delta parameter. This will directly affect the position
    of the cosine shift parameter. In this case, there is only two parameters
    which is the wing selection and the value of shift from centre position. 
    This means that either the delta is shifted from middle (normal cosine) 
    forward or backward. 
    This is to say that the wing has either a more or less aggressive split
    cycle cosine.
    
    wingside is 0 or 1 (selecting either one wing or the other)
*/
// int set_delta(int wingside, double nDelta)
// {
//     if(nDelta < Omega[wingside]){ // Delta can't be greater than Omega
//         // adjust delta's for SPI transfer
//         uint16_t iOmega = Omega[wingside]*100;
//         uint16_t iDelta = nDelta*100;
//         uint16_t sign = 0; // orientation of delta (POS or NEG)
//         if(nDelta < 0) {
//             iDelta = nDelta*(-100);
//             sign = 1; 
//         } 
//         else {
//             iDelta = nDelta*100;
//             sign = 0;
//         }
        
//         //Delta = nDelta;
//         int TABLE_S = 0;
//         int SPI_S = 0;

//         int r = -1;
//         while(r < 0){
//             r = setShortParams(wingside, TABLE_S, SPI_S, iOmega, iDelta, sign);
//         }   
//         if(r == 0){ // when sent success, set global Omega and Delta
//             Delta[wingside] = nDelta;
//         }
//         return r;
//     }
//     else{
//         printf("Can't send: Delta greater than Omega, D: %f, O: %f \n", nDelta, Omega[wingside]); // can also return 1 if needed
//         return 1;
//     } 
//     return 1; // if this happens then wow
// }

int set_delta_b(int wingside, double nDelta, int ActPas)
{
    if(nDelta < Omega[ActPas][wingside]){ // Delta can't be greater than Omega
        // adjust delta's for SPI transfer
        uint16_t iOmega = Omega[ActPas][wingside]*100;
        uint16_t iDelta = nDelta*100;
        uint16_t sign = 0; // orientation of delta (POS or NEG)
        if(nDelta < 0) {
            iDelta = nDelta*(-100);
            sign = 1; 
        } 
        else {
            iDelta = nDelta*100;
            sign = 0;
        }
        
        //Delta = nDelta;
        int TABLE_S = 0;
        int SPI_S = ActPas;

        int r = -1;
        while(r < 0){
            r = setShortParams(wingside, TABLE_S, SPI_S, iOmega, iDelta, sign);
        }   
        if(r == 0){ // when sent success, set global Omega and Delta
            Delta[ActPas][wingside] = nDelta;
        }
        return r;
    }
    else{
        printf("Can't send: Delta greater than Omega, D: %f, O: %f \n", nDelta, Omega[ActPas][wingside]); // can also return 1 if needed
        return 1;
    } 
    return 1; // if this happens then wow
}

int set_direction(int dir, int Wing){
    uint16_t GARBAGE = 5555;
    uint16_t COMMAND = -1; //will make it 10 for now
    if(dir == 0) COMMAND = 100;
    else if(dir == 1) COMMAND = 101;
    uint16_t cmd = 0; // cmd will be previous sent command as recieved from slave device
    int spiDevice = -1;

    // struct timespec time;
    // time.tv_sec = 0;
    // time.tv_nsec = 1000000L; // 1millisec

    if(Wing == 0){
        spiDevice = spi1d; // hopefully this works
        GPIOwrite(144, 0);  // left wing SPI_S
        GPIOwrite(145, 0); // left wing TABLE_S must be inverted from SPI_S
    } else if(Wing == 1){
        spiDevice = spi2d; //  hopefully this works
        GPIOwrite(146, 0); //  right wing SPI_S
        GPIOwrite(147, 0); // right wing TABLE_S must be inverted from SPI_S
    } 

    if(COMMAND < 0) return(-1);
    if(spiDevice < 0) return(-1); // this means that the wing command that came in was a fail
    cmd = SPItx16b(spiDevice, COMMAND); // send command for 16 bit transfer, recieve cmd which will be the previous sent message. cmd here should be grabage.
    nanosleep(&ti, NULL);
    cmd = SPItx16b(spiDevice, GARBAGE); // this cmd should be == COMMAND
    nanosleep(&ti, NULL);
    if(cmd != COMMAND){
        fprintf(stderr, "Returned command from previous message not same as intended command!\n");
        sendBroken(spiDevice);
        return (-1);
    } else printf("%s\n", "Sent command to chdir");
    return (0);
}

int set_Table(uint32_t *tTable, int size, int wing){
//http://stackoverflow.com/questions/23547544/how-do-i-set-an-array-equal-to-the-return-value-of-a-char-method
//http://www.tutorialspoint.com/cprogramming/c_pointer_to_an_array.htm
//http://stackoverflow.com/questions/1106957/passing-an-array-by-reference-in-c
    
    if(wing == 0){
        Table0 = calloc(size, sizeof(uint32_t));
        memcpy(Table0, tTable, size*sizeof(uint32_t));
    }
    else if(wing == 1){
        Table1 = calloc(size, sizeof(uint32_t));
        memcpy(Table1, tTable, size*sizeof(uint32_t));
    }
    // Table = calloc(size, sizeof(uint32_t));
    // //uint32_t t[size];
    // //int i = 0;
    // memcpy(Table, tTable, size*sizeof(uint32_t));
    //Table = t;
    // for (i = 0; i < size; ++i)
    // {
    //     printf("%"PRIu32",%d\n", Table[i], i);
    // }
    return (0);
}

/*
    This will take a new commutation table that details the shape of movement 
    that the wing will experience. This is to say that if the caller wants 
    to use a wing movement shape that has elaborate movement, then this 
    fucntion will transfer the table the details that shape to the vehicle
    wing which will then override the existing table with that new table.
*/
int send_commutation(int wingside)
{
    int r = -1;
    if(wingside == 0){
        r = setLongParams(wingside, activContr[wingside], !activContr[wingside], Table0);
    }
    else if(wingside == 1){
        r = setLongParams(wingside, activContr[wingside], !activContr[wingside], Table1);
    }
    
    if(r >= 0) printf("%s\n", "Sent Table");
    else {
        printf("%s\n", "Commit failed");
        return -1;
    }
    return 0;
}

int switch_control(){

    GPIOwrite(144, !activContr[0]);
    GPIOwrite(145, !activContr[0]);
    GPIOwrite(146, !activContr[1]);
    GPIOwrite(147, !activContr[1]);

    // if(ActPas == 0){
    //     GPIOwrite(144, 0); // left wing TABLE_S must be inverted from SPI_S
    //     GPIOwrite(145, 0); // left wing TABLE_S must be inverted from SPI_S
    // }else if(ActPas == 1){
    //     GPIOwrite(144, 1); // left wing TABLE_S must be inverted from SPI_S
    //     GPIOwrite(145, 1); // left wing TABLE_S must be inverted from SPI_S
    // }
    
    return 0;
}

int en_dis(int COMMAND)
{
    // printf("PIC# for SPI Select:   ");
    // scanf("%d", &SPI_S);
    // printf("PIC# for TABLE Select: ");
    // scanf("%d", &TABLE_S);
    // //TABLE_S = SPI_S;
    // //first make the selected pic as SPI "passive" PIC
    // GPIOwrite(144, SPI_S);  // disA or controlA etc
    // GPIOwrite(145, TABLE_S);
    // //GPIOwrite(146, SPI_S);
    // //GPIOwrite(147, TABLE_S);

    //spi1d = SPIinit(device1);
    //spi2d = SPIinit(device2);
    //while(1){
    SPItx(spi1d, COMMAND);
    SPItx(spi2d, COMMAND);
    //}
    return 0;
}




int free_memory(){
    free(Table0);
    free(Table1);
    return 0;
}

/*
int switch_seq()
{
    int SPI_S = 0;
    
    printf("PIC# for SPI Select: ");
    scanf("%d", &SPI_S);
    //printf("PIC# for TABLE Select: ");
    //scanf("%d", &TABLE_S);
    //TABLE_S = SPI_S;

    //finally, make the selected PIC as TABLE "active" PIC
    //GPIOactivelh(144, SPI_S);
    //GPIOactivelh(145, TABLE_S);

    GPIOwrite(144, SPI_S);  // left wing SPI_S
    GPIOwrite(145, SPI_S); // left wing TABLE_S must be inverted from SPI_S
    GPIOwrite(146, SPI_S); //  right wing SPI_S
    GPIOwrite(147, SPI_S); // right wing TABLE_S must be inverted from SPI_S
    
    return 0;
}
*/

/*
int freq_seq()
{
    int SPI_S = 0;
    uint16_t D1 = 2512;
    uint16_t D2 = 628;
    uint16_t dir = 0;
    
    printf("PIC# for SPI for Frequency Set: ");
    scanf("%d", &SPI_S);
    //printf("PIC# for TABLE Frequency Set: ");
    //scanf("%d", &TABLE_);
    printf("Delta 1: ");
    scanf("%hu", &D1);
    printf("Delta 2: ");
    scanf("%hu", &D2);
    printf("Direction: ");
    scanf("%hu", &dir);
    //printf("Here: %hu \n", D1);
    setShortParams(0, SPI_S, SPI_S, D1, D2, dir); // wingR = 1, 
    setShortParams(1, SPI_S, SPI_S, D1, D2, dir); // wingL = 0, 
        
    return 0;
}
*/

