#include "font.h"

void drawChar(unsigned char x, unsigned char y, unsigned char character) {
    int row,col;
    for ( col = 0; col < 5; col++ ) {
        for ( row = 0; row < 8; row++ ) {
            if (((ASCII[character-0x20][col]) >> row) & 1) {
                ssd1306_drawPixel(x+col,y+row,1);
            }
            else {
                ssd1306_drawPixel(x+col,y+row,0);
            }
        }
    }
}

void drawString(unsigned char x, unsigned char y, unsigned char *message) {
    int i=0; // for counting message number
    while(message[i] != 0) {
        drawChar(x,y,message[i]);
        x = x+5;
        i++;
    }
    
}