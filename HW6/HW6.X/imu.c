#include "imu.h"
#include "i2c_master_noint.h"

void imu_setup(){
    unsigned char who = 0;

    // read from IMU_WHOAMI
    
    i2c_master_start(); // starts 12c communication
    i2c_master_send(0b11010110); // addresses the device to send information to //need to append write
    i2c_master_send(0x0F); // selects the pin to read from
    i2c_master_restart();
    i2c_master_send(0b11010111); // addresses the device to send information to //need to append read
    who = i2c_master_recv(); // get received values of all 8 pins
    i2c_master_ack(1); // acknowledges the value has been received
    i2c_master_stop(); // ends i2c communication
    
    if (who != 0b01101001){ //basically, the setup fails
        i2c_master_start();
        i2c_master_send(0b01000000); // selects MCP as slave target
        i2c_master_send(0x01); // selects B pins for I/O
        i2c_master_send(0xFF); // sets B pins as O/P
        i2c_master_stop();
        
        while(1){ // flashes yellow LED
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 24000000/10 ) { // 10Hz delay
                i2c_master_start(); // starts 12c communication
                i2c_master_send(0b01000000); // addresses the device to send information to //need to append write
                i2c_master_send(0x14); // selects the pin to send information to
                i2c_master_send(0xFF); // information for pin to receive
                i2c_master_stop(); // ends i2c communication
            }
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 24000000/10 ) { // 10Hz delay
                i2c_master_start(); // starts 12c communication
                i2c_master_send(0b01000000); // addresses the device to send information to //need to append write
                i2c_master_send(0x14); // selects the pin to send information to
                i2c_master_send(0x00); // information for pin to receive
                i2c_master_stop(); // ends i2c communication
            }
        }
    }

    // init IMU_CTRL1_XL
    i2c_master_start();
    i2c_master_send(0b11010110); // selects IMC as slave target to write
    i2c_master_send(0x10); // selects XL for I/O 
    i2c_master_send(0b10000010); // sets sample rate to 1.66 kHz, 2g sensitivity, 100 Hz filter
    i2c_master_stop();
    
    
    // init IMU_CTRL2_G
    i2c_master_start();
    i2c_master_send(0b11010110); // selects IMC as slave target to write
    i2c_master_send(0x11); // selects G for I/O 
    i2c_master_send(0b10001000); // sets sample rate to 1.66 kHz, 1000 dps sensitivity.
    i2c_master_stop();
    
    // init IMU_CTRL3_C
    i2c_master_start();
    i2c_master_send(0b11010110); // selects IMC as slave target to write
    i2c_master_send(0x12); // selects C for I/O 
    i2c_master_send(0b00000100); // Makes IF_INC=1 - read multiple registers in a row w/o specifying all register locations
    i2c_master_stop();
    
}

void imu_read(unsigned char reg, signed short * data, int len){
    
    // read multiple from the imu, each data takes 2 reads so you need len*2 chars
    unsigned char address = 0b11010110; // address of imu
    int length = len * 2; // number of communications needed
    unsigned char datainput[length]; // array of data from communications
    i2c_master_read_multiple(address, reg, datainput, length); // gets data from the imu

    // turn the chars into the shorts
    int highbit;
    for (highbit = 2; highbit <= length; highbit=highbit+2 ) { //i is the first  value of datainput
        int lowbit = highbit-1;                                //j is the second value of datainput
        int outbit = (highbit)/2;                              //k is the value to put in dataoutput
        data[outbit] = datainput[highbit] << 8 | datainput[lowbit];
    }
    
//    data[0] = datainput[0] << 8 | datainput[1];
//    data[1] = datainput[2] << 8 | datainput[3];
//    data[2] = datainput[4] << 8 | datainput[5];
//    data[3] = datainput[6] << 8 | datainput[7];
//    data[4] = datainput[8] << 8 | datainput[9];
//    data[5] = datainput[10] << 8 | datainput[11];
//    data[6] = datainput[12] << 8 | datainput[13];
}