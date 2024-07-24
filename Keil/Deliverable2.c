  // Design Project Deliverable 2
  // For: COMPENG 2DX3 McMaster University, Winter 2023, Design Project
  // By: Yash Bhatia | 28th March, 2023
  // Last Modified: 12th April, 2023

  // Using code by the 2DX3 Instructional/Teaching Team and Valvano et al. from the Embedded Systems: Introduction to Arm Cortex M Microcontrollers textbook.
	// All credits preserved in source files.
  // Using libraries provided by STMicroelectronics for the VL53L1X ToF sensor (Ultra Light Driver).
  // Using code co-written with Ria Saldanha (saldanhr) for Lab 6.

  /*  NOTES
  *   Bus speed based on table- 1st LSB of my student number is 2
  *   This means the bus speed is 24 MHz
  *   Current bus speed is 120 MHz - we must reduce by a factor of 5
  *   This means we must divide by 5. This is done by setting PSYSDIV to 19. See PLL.h
  *   Because of new bus speed, Systick.c has been modified to reflect this. See Systick.c
  *
  *   Pin Assignments based on table- 2nd LSB of my student number is 7
  *   My pin assignments are: 
  *   PF4 for Measurement LED
  *   PF0 for Additional LED
  *   Both of these are on the board! PF4- D3. PF0- D4.
  *   
  *   Other pins are:
  *   PB2: I2C0 SCL for ToF sensor
  *   PB3: I2C0 SDA for ToF sensor
  *   PA0-1: UART0
  *   PE0-3: Motor Driver. I don't use port H because it is inconvenient to split the pins up.
  *   PJ1: Pushbutton. Will be set in ACTIVE LOW mode. This means that when the button is pressed, the pin will be LOW. Using pull-up resistor.
  *
  *		TODO: None, codebase complete. 
  */

  #include <stdint.h>
  #include "PLL.h"
  #include "SysTick.h"
  #include "uart.h"
  #include "onboardLEDs.h"
  #include "tm4c1294ncpdt.h"
  #include "VL53L1X_api.h"

  // Some common constants for I2C

  #define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
  #define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
  #define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
  #define I2C_MCS_STOP            0x00000004  // Generate STOP
  #define I2C_MCS_START           0x00000002  // Generate START
  #define I2C_MCS_ERROR           0x00000002  // Error
  #define I2C_MCS_RUN             0x00000001  // I2C Master Enable
  #define I2C_MCS_BUSY            0x00000001  // I2C Busy
  #define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

  #define MAXRETRIES              5           // number of receive attempts before giving up
  void I2C_Init(void){
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
    while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

      GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
      GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

      GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
  //    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                              // 6) configure PB2,3 as I2C
  //  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
      I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
      I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
  //    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
          
  }

  int delay = 1;
  uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
  int status=0;

    uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
    uint16_t wordData;
    uint16_t Distance[256];
    uint16_t SignalRate;
    uint16_t AmbientRate;
    uint16_t SpadNum; 
    uint8_t RangeStatus;
    uint8_t dataReady;
      

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

  void PortJ1_Init(void)
  {
      // initialise Port J pin 1 (onboard push button)
      // to be used by pushbutton, make sure to use pull-up resistor
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; // enable clock to GPIOJ
      while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {}; // wait for clock to stabilise
      // set PJ1 as digital pin
      GPIO_PORTJ_DEN_R = 0x2;
      // set PJ1 as input to read pushbutton
      GPIO_PORTJ_DIR_R = 0x0;
			GPIO_PORTJ_PUR_R = 0x2;
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
	void motor_rotate_opposite(int step, int delay);
  int main(void);

  void runState(void) // this will be the function that will sense data
  {
			FlashAllLEDs(); // indicate that the program is running

      while(sensorState==0){
      status = VL53L1X_BootState(dev, &sensorState); // check if sensor is booted
      SysTick_Wait10ms(10);
      }

      status = VL53L1X_SensorInit(dev); // Intialise the sensor, the settings are in the Ultra Light Driver API
      status = VL53L1X_SetDistanceMode(dev, 2); // Set long range mode

      status = VL53L1X_StartRanging(dev);   // Write configuration bytes to initiate measurement



      for (int i = 0; i < 256; i++) // rotate motor 256 times, 1.4 degrees each time. a new measurement is taken every 20 ms
      {
          motor_rotate(2, delay); // rotate motor 2 steps

          // time to gather data

          while (dataReady == 0){
            status = VL53L1X_CheckForDataReady(dev, &dataReady); // Check if new data is available
            FlashLED3(1); // flash the LED to indicate that the sensor is gathering data
            VL53L1_WaitMs(dev, 5);
          }
          dataReady = 0;

          status = VL53L1X_GetDistance(dev, &Distance[i]); // Read and store the measurement into the array
          status = VL53L1X_ClearInterrupt(dev); // Clear interrupt before next measurement is called

          SysTick_Wait10ms(1); // wait 20 milliseconds before taking the next measurement

      } 

      status = VL53L1X_StopRanging(dev); // all measurements are done, stop ranging
			
			for (int i = 0; i < 256; i++) // counter-clockwise rotation: take the motor back to its original position
      {
          motor_rotate_opposite(2, delay); 

          SysTick_Wait10ms(1); // faster since we don't need to gather data

      } 

      // print out the distance measurements
      GPIO_PORTF_DATA_R = 0b1; // turn on LED to indicate that the motor has finished rotating
      for (int i = 0; i < 256; i++)
      {
					sprintf(printf_buffer, "%d\r\n", Distance[i]); 
          UART_printf(printf_buffer); // transmit the data to the computer via UART
      }
      GPIO_PORTF_DATA_R = 0b0; // turn off LED to indicate that transmission is done

      SysTick_Wait10ms(3);

      return;  // return to wait state
  }

  void waitState(void) // this will be the function that will wait for the pushbutton to be pressed
  {
			GPIO_PORTE_DATA_R = 0x0; // turn off the motor when in wait state to prevent magnets from being on for long!
      while (1) // wait for PJ1 to be pressed to turn motor back on
      {  
          if (!(GPIO_PORTJ_DATA_R & 0x02)) // motor restarts if PJ1 is pressed
          {	
							while (!(GPIO_PORTJ_DATA_R & 0x02)) {} // wait for PJ1 to be released
              runState(); // run the motor
          }
      }
  }


  void motor_rotate(int step, int delay)
  {
      // rotate motor by 'step' steps, with a delay of 'delay' x 5ms between each step
      for (int i = 0; i < step; i++)
      {
        GPIO_PORTE_DATA_R = 0b00001001;
        SysTick_Wait10us(delay*500);
        GPIO_PORTE_DATA_R = 0b00000011;
        SysTick_Wait10us(delay*500);
        GPIO_PORTE_DATA_R = 0b00000110;
        SysTick_Wait10us(delay*500);
        GPIO_PORTE_DATA_R = 0b00001100;
        SysTick_Wait10us(delay*500);
      }
      return;
  }
	
	void motor_rotate_opposite(int step, int delay) // same as the function above, but in the opposite direction
  {
      
      for (int i = 0; i < step; i++)
      { 
        GPIO_PORTE_DATA_R = 0b00001100;
        SysTick_Wait10us(delay*500);
        GPIO_PORTE_DATA_R = 0b00000110;
        SysTick_Wait10us(delay*500);
        GPIO_PORTE_DATA_R = 0b00000011;
        SysTick_Wait10us(delay*500);
        GPIO_PORTE_DATA_R = 0b00001001;
        SysTick_Wait10us(delay*500);
      }
      return;
  }
	



  int main (void)
  {
  //		uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  //  uint16_t wordData;
  //  uint16_t Distance;
  //  uint16_t SignalRate;
  //  uint16_t AmbientRate;
  //  uint16_t SpadNum; 
  //  uint8_t RangeStatus;
  //  uint8_t dataReady;
      
      // initialise everything
      PLL_Init(); // set system clock to 120 MHz
      SysTick_Init(); // initialise systick timer
      PortF0F4_Init(); // initialise Port F pins 0 and 4
      PortJ1_Init(); // initialise Port M pins 0 and 1
      PortE0E1E2E3_Init(); // initialise Port E pins 0, 1, 2, and 3
      UART_Init();
      onboardLEDs_Init();
      I2C_Init();

      waitState(); // start in wait state
  }