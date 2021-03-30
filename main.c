/*******************************************************************************
*                               MSP432 Encoder                                 *
*                                                                              *
* Author:  Long Tran                                                           *
* Device:  MSP432P401R LaunchPad                                               *
* Program: Display encoder turn count on 7-segment display                     *
*                                                                              *
* Demo: https://www.youtube.com/watch?v=wDcu66NudCA                            *
*******************************************************************************/

#include "msp.h"
#include "stdlib.h"

void sseg_modulo(void);     // divide counter into digits using modulo operator
void sseg_display(void);    // display counter digits on 7-segment display
void wait(uint32_t t);

const uint8_t look_up[10] = {   // 7-segment display look up table
0b11000000,  // 0
0b11111001,  // 1
0b10100100,  // 2
0b10110000,  // 3
0b10011001,  // 4
0b10010010,  // 5
0b10000010,  // 6
0b11111000,  // 7
0b10000000,  // 8
0b10010000,  // 9
};

volatile int16_t counter;   // encoder angle counter
uint16_t temp;
uint8_t display[4] = {0,0,0,0}; // 7-seg display array

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    //-- Configure NVIC
    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31); //enable P3 and P5 interrupt
    NVIC->ISER[1] = 1 << ((PORT5_IRQn) & 31);
    //-- Initialize Clock

    //-- Configure Encoder
    P3->DIR &= ~BIT6;   // P3.6 phaseA input 
    P3->IE  |= BIT6;
    P5->DIR &= ~BIT3;   // P5.3 phaseB input
    P4->IE  |= BIT3;

    //-- Configure 7-Seg Display
    P4->DIR = 0xFF;  // P4 is 7-segment LED output
    P8->DIR = 0xFF;  // P8 is display output
    P5->DIR |= BIT0; // P5.0 is red LED angle polarity indicator

    while(1){

        if(counter > 0){
            P5->OUT &= ~BIT0;   // red LED off, positive angle (CW)
        }else{
            P5->OUT |= BIT0;    // red LED on, negative angle (CCW)
        }

        temp = abs(counter)%9999;

        sseg_modulo();
        sseg_display();
    }

}

//-- Functions
void sseg_modulo(void){

    display[0] = temp/1000;
    display[1] = (temp/100)%10;
    display[2] = (temp/10)%10;
    display[3] = temp%10;
}

void sseg_display(void){

    static uint8_t k = 0;

        // Display digit-k
        P4->OUT = 0xFF;                      // blank 7-seg display
        P8->OUT = 0xFF & ~(BIT5 >> k);       // enable k-th digit in 7-seg display
        P4->OUT = look_up[display[k]];                // display k-th digit in 7-seg display

        // increment k index
        k++;
        if (k >= 4){k = 0;}

        // reduce flickering
        wait(1000);
}

void wait(uint32_t t){
    while(t > 0){t--;}
}

//-- Interrupts
void PORT3_IRQHandler(void){
    if(P3->IV & 0x0E){      // if phaseA is interrupt (rising edge)
        if(P5->IN & BIT3){  // if phaseB is high
            counter--;
        }else{
            counter++;
        }
    }
}

void PORT5_IRQHandler(void){
    if(P5->IV & 0x08){      // if phaseB is interrupt (rising edge)
        if(P3->IN & BIT6){  // if phaseA is high
            counter--;
        }else{
            counter++;
        }
    }
}
