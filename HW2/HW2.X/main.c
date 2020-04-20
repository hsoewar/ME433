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

void initSPI();
unsigned char spi_io(unsigned char o);

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
    TRISBbits.TRISB4 = 1;
    
    initSPI();

    __builtin_enable_interrupts();
    
    unsigned char c = 0; // c bit
    

    // V bits
    unsigned short V[2][100]; // array with 2 rows, 100 columns
    int i,j;
    
    for ( i = 0; i < 2; i++ ) {
        if (i==0) { // sine wave
            for ( j = 0; j < 100; j++ ) {
                V[i][j] = (sin((j*4*3.14159)/(100))+1) * (4096/2); //check
            }
        }
        if (i==1) { // triangle wave
            for ( j = 0; j < 100; j++ ) {
                if (j<=50) {
                    V[i][j] = 4095*j/50;
                }
                else {
                    V[i][j] = V[i][100-j];
                }
                
            }
        }
    }

    
    unsigned short p;
    unsigned char k=0;

    
    while (1) {
        
        /*while (c < 2) { //outputs one value of both the sine and triangle waves
            p = (c<<15);
            p = p|(0b111<<12);
            p = p|V[c][k];

            // write two bytes over SPI
            LATAbits.LATA0 = 0; // bring CS low
            spi_io(p>>8); // write the first byte
            spi_io(p); //write the second byte
            LATAbits.LATA0 = 1; // bring CS high
            
            c++;
        }*/
        
        p = (c<<15);
        p = p|(0b111<<12);
        p = p|V[c][k];

        // write two bytes over SPI
        LATAbits.LATA0 = 0; // bring CS low
        spi_io(p>>8); // write the first byte
        spi_io(p); //write the second byte
        LATAbits.LATA0 = 1; // bring CS high
        
        c++;
        
        p = (c<<15);
        p = p|(0b111<<12);
        p = p|V[c][k];

        // write two bytes over SPI
        LATAbits.LATA0 = 0; // bring CS low
        spi_io(p>>8); // write the first byte
        spi_io(p); //write the second byte
        LATAbits.LATA0 = 1; // bring CS high

        c = 0;

        k++; // moves onto the next step in the sine and triangle waves
        if (k == 100) {
            k = 0;
        }
        
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 24000000/100 ) { // 100Hz delay
        }
        
    }
}

// initialize SPI1
void initSPI() {
    // Pin B14 has to be SCK1
    // Turn off all A analog pins
    ANSELA = 0; // 1 for analog
    // Make an output pin for CS
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 1;
    // Set SDO1
    RPA1Rbits.RPA1R = 0b0011;
    // Set SDI1
    SDI1Rbits.SDI1R = 0b0001;

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1000; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
}

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;
    while (!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;
}