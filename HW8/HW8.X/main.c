#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>        // for sprintf
#include "imu.h"         // for accelerometer/gyroscope/temperature
#include "ws2812b.h"     // for colorshange LEDs
#include "rtcc.h"        // for rtccTime struct

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = ON // enable secondary oscillator
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
void ws2812b_setup();
rtccTime readRTCC();

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
    heartbeat_setup();  //sets pin for heartbeat LED flashing    
    i2c_master_setup(); // initializes I2C connection
    ssd1306_setup();    // initializes OLED screen
    initI2C();          // sets pin for failure LED
    ws2812b_setup();    //sets up LEDs
    adc_setup();        // sets up analog to digital converter
    ctmu_setup();       // sets up capacitive touch
    unsigned long time = 0x14380000; // Time set to 11:05:00
    unsigned long date = 0x20052804; // Date set to Ths 20200528
    rtcc_setup(time, date);

        
    __builtin_enable_interrupts();
    
    int counter = 0;
    rtccTime readtime;
    char day[11];
    
    while (1) {
        readtime = readRTCC();
        
        dayOfTheWeek(readtime.wk, day);
        
        heartbeat();
        
        ssd1306_clear();
        
        char message[50]; // string to be printed on the OLED
        sprintf(message, "Am I working? %d", counter);
        drawString(0, 0, message);
        sprintf(message, "%d%d:%d%d:%d%d", readtime.hr10, readtime.hr01, readtime.min10, readtime.min01, readtime.sec10, readtime.sec01);
        drawString(0, 16, message);
        sprintf(message, "%s, %d%d/%d%d/20%d%d", day, readtime.mn10, readtime.mn01, readtime.dy10, readtime.dy01, readtime.yr10, readtime.yr01);
        drawString(0, 24, message);

        ssd1306_update();
        
        counter = counter+1;
        
        while (_CP0_GET_COUNT() < 48000000 / 2 / 2  ) { // 1Hz delay
    }

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
    TRISBbits.TRISB5 = 0; // sets as digital output
    LATBbits.LATB5 = 0; // sets output to 0 (off)
}

// heartbeat LED
void heartbeat() {
    _CP0_SET_COUNT(0);
    LATBbits.LATB5 = !LATBbits.LATB5; // inverts output (i.e. 1 becomes 0, 0 becomes 1)
    
}
