#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>        // for sprintf
#include "imu.h"

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1 // use largest wdt ---CHECK!!!
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

void initI2C(); // basically, this is for the failure LED, I think
void setPin(unsigned char address, unsigned char register_1, unsigned char value);
void heartbeat_setup(); // because otherwise there's some warning, even though the thing runs fine
void heartbeat();
void bar_x(signed short x);
void bar_y(signed short y);


int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    heartbeat_setup(); //sets pin for heartbeat LED flashing    
    i2c_master_setup(); // initializes I2C connection
    ssd1306_setup(); // initializes OLED screen
    initI2C(); // sets pin for failure LED
    imu_setup(); // sets up imu
    
    __builtin_enable_interrupts();
        
    int len = 7; // number of communications needed
    signed short data[len]; // array of 7 values from the imu
    
    
    while (1) {
        heartbeat();
        
        imu_read(IMU_OUT_TEMP_L, data, len);
        
        ssd1306_clear();
        
        // high level troubleshooting: plots each collected output from the imu
//        char message[50]; // string to be printed on the OLED
//        sprintf(message, "g: %d %d %d", data[2], data[3], data[4]);
//        drawString(0, 0, message);
//        sprintf(message, "a: %d %d %d", data[5], data[6], data[7]);
//        drawString(0, 8, message);
//        sprintf(message, "t: %d", data[1]);
//        drawString(0, 16, message);
        
        // deep level troubleshooting: uncomment to see the each individual output from the imu
//        unsigned char address = 0b11010110; // address of imu
//        int length = len * 2; // number of communications needed
//        unsigned char datainput[length]; // array of data from communications
//        i2c_master_read_multiple(address, IMU_OUT_TEMP_L, datainput, length); // gets data from the imu
//
//        char message[50]; // string to be printed on the OLED
//        sprintf(message, "%d %d %d %d %d %d", datainput[4], datainput[3], datainput[6], datainput[5], datainput[8], datainput[7]);
//        drawString(0, 0, message);
//        sprintf(message, "%d %d %d %d %d %d", datainput[10], datainput[9], datainput[12], datainput[11], datainput[14], datainput[13]);
//        drawString(0, 8, message);
//        sprintf(message, "%d %d", datainput[2], datainput[1]);
//        drawString(0, 16, message);
        
        // bar thingie 
        bar_x(data[5]);
        bar_y(-data[6]);
        
        
        ssd1306_update();
    }
}

// initialize I2C
void initI2C() {
    i2c_master_setup();
    
    i2c_master_start();
    i2c_master_send(0b01000000); // selects MCP as slave target
    i2c_master_send(0x00); // selects A pins for I/O 
    i2c_master_send(0x00); // sets A pins as I/P
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0b01000000); // selects MCP as slave target
    i2c_master_send(0x01); // selects B pins for I/O
    i2c_master_send(0xFF); // sets B pins as O/P
    i2c_master_stop();
}

// writes to I2C A pins
void setPin(unsigned char address, unsigned char register_1, unsigned char value){
    i2c_master_start(); // starts 12c communication
    i2c_master_send(0b01000000); // addresses the device to send information to //need to append write
    i2c_master_send(register_1); // selects the pin to send information to
    i2c_master_send(value); // information for pin to receive
    i2c_master_stop(); // ends i2c communication
    
}

// reads from I2C A pins
unsigned char readPin(unsigned char address, unsigned char register_2){
    i2c_master_start(); // starts 12c communication
    i2c_master_send(0b01000000); // addresses the device to send information to //need to append write
    i2c_master_send(register_2); // selects the pin to read from
    i2c_master_restart();
    i2c_master_send(0b01000001); // addresses the device to send information to //need to append read
    unsigned char data = i2c_master_recv(); // get received values of all 8 pins
    i2c_master_ack(1); // acknowledges the value has been received
    i2c_master_stop(); // ends i2c communication
    return data; // brings data to main function
}

//heartbeat LED setup
void heartbeat_setup() {
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
}

// heartbeat LED
void heartbeat() {
    _CP0_SET_COUNT(0);
    LATAbits.LATA4 = !LATAbits.LATA4;
    while (_CP0_GET_COUNT() < 48000000 / 2 / 20 ) { // 20Hz delay
    }
}

// x-accelerometer reading to x-movement
void bar_x(signed short x) {
    signed int bar_mag = x  * 32 / 32767;
    int i;
    if (bar_mag<0) {
        for (i = 0; i >= bar_mag ; i=i-1) {
            ssd1306_drawPixel(128/2, 32/2+i, 1);
        }
    }
    else {
        for (i = 0; i <= bar_mag; i=i+1) {
            ssd1306_drawPixel(128/2, 32/2+i, 1);
        }
    }
}

// y-accelerometer reading to y-movement
void bar_y(signed short y) {
    signed int bar_mag = y * 128 / 32767;
    int i;
    if (bar_mag<0) {
        for (i = 0; i >= bar_mag ; i=i-1) {
            ssd1306_drawPixel(128/2+i, 32/2, 1);
        }
    }
    else {
        for (i = 0; i <= bar_mag; i=i+1) {
            ssd1306_drawPixel(128/2+i, 32/2, 1);
        }
    }
}