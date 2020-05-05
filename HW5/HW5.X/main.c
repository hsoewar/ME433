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

typedef struct { 
    unsigned char r;
    unsigned char g;
    unsigned char b;
} wsColor; // defines the wsColor array to be used in everything
    
void heartbeat_setup(); // for LED heartbeat
void heartbeat();

wsColor HSBtoRGB(float hue, float sat, float brightness); // because otherwise C thinks HSBtoRGB should output an int

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
    heartbeat_setup();
    
    ws2812b_setup(); //sets up LEDs
    
    __builtin_enable_interrupts();
    
    wsColor LED1; //defines wsColor structure for LEDs
    wsColor LED2;
    wsColor LED3;
    wsColor LED4;
    float i = 0; // count for hue
    float j;
    float k;
    float l;
    float n = 60; // increment between led hues
    float sat = 1; //not at all gray
    float bright = 0.25; // so I don't go blind
    
    while (1) {
        //heartbeat();
        
        j = i+n;
        k = j+n;
        l = k+n;
        
        if (i >= 360) {
            i=i-360;
        }
        if (j >= 360) {
            j=j-360;
        }
        if (k >= 360) {
            k=k-360;
        }
        if (l >= 360) {
            l=l-360;
        }
        
        LED1 = HSBtoRGB(i, sat, bright);
        LED2 = HSBtoRGB(j, sat, bright);
        LED3 = HSBtoRGB(k, sat, bright);
        LED4 = HSBtoRGB(l, sat, bright);
        
        wsColor c[4] = {LED1, LED2, LED3, LED4}; // sets some color..?
        int numLEDs = 4; // 4 LEDs used
        ws2812b_setColor(c,numLEDs);
        
        i = i + 0.1;
        

        /* // basic setup of LEDs using hex
        wsColor LED1 = {0xFF, 0xFF, 0xFF};
        wsColor LED2 = {0xFF, 0x00, 0x00};
        wsColor LED3 = {0x00, 0xFF, 0x00};
        wsColor LED4 = {0x00, 0x00, 0xFF};
        
        wsColor c[4] = {LED1, LED2, LED3, LED4}; // sets some color..?
        
        int numLEDs = 4; // 4 LEDs used
        ws2812b_setColor(c,numLEDs);*/
        
    }
}

//heartbeat LED setup
void heartbeat_setup() {
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
}


// heartbeat LED
void heartbeat() {
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < 48000000 / 2 / 2 ) { // 2Hz delay
        LATAbits.LATA4 = 0;
    }
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < 48000000 / 2 / 2 ) { // 2Hz delay
        LATAbits.LATA4 = 1;
    }
}
