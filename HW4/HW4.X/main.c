#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

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

void initI2C();
void ssd1306_setup();
void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color);
void ssd1306_update();

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
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;

    initI2C(); // initializes I2C connection
    ssd1306_setup(); // initializes OLED screen
    
    __builtin_enable_interrupts();
    
    while (1) {
        
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 24000000/10 ) { // 10Hz delay
            LATAbits.LATA4 = 0;
            ssd1306_drawPixel(0, 0, 1);
            ssd1306_update();
        }
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 24000000/10 ) { // 10Hz delay
            LATAbits.LATA4 = 1;
            ssd1306_drawPixel(0, 0, 0);
            ssd1306_update();
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