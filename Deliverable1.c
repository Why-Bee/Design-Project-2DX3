// Design Project Deliverable 1
// For: COMPENG 2DX3 McMaster University, Winter 2023, Design Project
// By: Yash Bhatia | 19th March, 2023
// Assignment:
// - Figure out pin assignments
// - Correctly wire up a motor and pushbutton, choose active Lo or Hi
// - Once the button is pressed, start rotating the motor 45 degrees at a time clockwise. Blink an LED everytime the motor reaches 45 degrees
// - If the button is pressed again, turn off the motor
// - If the motor is off and the button is pressed again, turn the motor back on and do the same thing
// - If, while the motor is running, the button is not pressed and a 360 degree rotation is achieved, stop the motor anyways.

// Using code by the 2DX3 Instructional/Teaching Team, from Studio 5. 
// Using code co-written with Ria Saldanha for Lab 6

/*  NOTES
*   Pin Assignments based on table- 2nd LSB of my student number is 7
*   My pin assignments are: 
*   PF4 for Measurement
*   PF0 for Additional
*   Both of these are on the board! PF4- D3. PF0- D4.
*
*		TODO: Better debouncing- make hardware filter? I heard RC filter is good for this application.
*/

#include <stdint.h>
#include "SysTick.h"
#include "PLL.h"
#include "tm4c1294ncpdt.h"

int delay = 1;

void PortF0F4_Init(void)
{
    // initialise Port F pins 0 and 4
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // enable clock to GPIOF
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {}; // wait for clock to stabilise
    // set PF4 and PF0 as digital pins
    GPIO_PORTF_DEN_R = 0x11; // 0x11 = 0001 0001
    // set PF4 and PF0 as output to drive LED
    GPIO_PORTF_DIR_R = 0x11; // 0x11 = 0001 0001
    return;
}

void PortM0_Init(void)
{
    // initialise Port M pin 0
    // to be used by pushbutton, make sure to use pull-up resistor
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; // enable clock to GPIOM
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0) {}; // wait for clock to stabilise
    // set PM0 as digital pin
    GPIO_PORTM_DEN_R = 0x01; // 0x01 = 0000 0001
    // set PM0 as input to read pushbutton
    GPIO_PORTM_DIR_R = 0x00; // 0x00 = 0000 0000
		// set pull up resistor on port PM0
		GPIO_PORTM_PUR_R = 0x01;
}

void PortE0E1E2E3_Init(void)
{
    // initialise Port E pins 0, 1, 2, and 3
    // outputs to the motor driver
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // enable clock to GPIOE
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0) {}; // wait for clock to stabilise
    // set PE0, PE1, PE2, and PE3 as digital pins
    GPIO_PORTE_DEN_R = 0x0F; // 0x0F = 0000 1111
    // set PE0, PE1, PE2, and PE3 as output to drive motor
    GPIO_PORTE_DIR_R = 0x0F;
    return;
}

void runState(void);
void waitState(void);
void motor_rotate(int step, int delay);

void runState(void)
{

    for (int i = 0; i < 8; i++) // rotate motor 8 times, 45 degrees each time
    {
        motor_rotate(64, delay); // rotate motor 64 steps, with a delay of 100 milliseconds between each step  
        GPIO_PORTF_DATA_R ^= 0x10; // toggle PF4 to blink LED
        SysTick_Wait10ms(35);
        GPIO_PORTF_DATA_R ^= 0x10; // toggle PF4 again to turn LED off
    }  

    return;  
}

void waitState(void)
{
    while (1) // wait for PM0 to be pressed to turn motor back on
    {
				GPIO_PORTE_DATA_R = 0x0; // turn off the motor when in wait state to prevent magnets from being on for long!
        if (!(GPIO_PORTM_DATA_R & 0x01)) // motor restarts if PM0 is pressed
        {
            while(!(GPIO_PORTM_DATA_R & 0x01)){}
            runState();
        }
    }
}


void motor_rotate(int step, int delay)
{
    // rotate motor by 'step' steps, with a delay of 'delay' milliseconds between each step
    for (int i = 0; i < step; i++)
    {
        // check if PM0 is pressed- cancel rotation if it is
        // remember PM0 is active low
        if (!(GPIO_PORTM_DATA_R & 0x01))
        {// polling every step is inefficient, but it works. might refactor to interrupts later
					while(!(GPIO_PORTM_DATA_R & 0x01)){}
          waitState();
        }
			GPIO_PORTE_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			GPIO_PORTE_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTE_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);
			GPIO_PORTE_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
    }
    return;
}



int main (void)
{
    // initialise everything
    PLL_Init(); // set system clock to 120 MHz
    SysTick_Init(); // initialise systick timer
    PortF0F4_Init(); // initialise Port F pins 0 and 4
    PortM0_Init(); // initialise Port M pins 0 and 1
    PortH0H1H2H3_Init(); // initialise Port H pins 0, 1, 2, and 3

    waitState();
}









