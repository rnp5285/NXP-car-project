/*
 * Main module for testing the PWM Code for the K64F
 * 
 * Author:  
 * Created:  
 * Modified: Carson Clarke-Magrab <ctc7359@rit.edu> 
 */

#include "MK64F12.h"
#include "pwm.h"

void delay(int del);
int main(void) {
	int  forward = 1;
	int  phase = 0;
	// Initialize UART and PWM
	// INSERT CODE HERE
InitDCMotPWM();
InitServoPWM();
SetDCMotDuty(50,50);
SetServoDuty(6);
	// Print welcome over serial
	//uart_put("Running... \n\r");
	
	 //Part 1 - UNCOMMENT THIS
	// Generate 20% duty cycle at 10kHz
	// INSERT CODE HERE
	//FTM0_set_duty_cycle(20,10000,1);
	//for(;;) ;  //then loop forever
//	FTM0_set_duty_cycle(20,10000,1);
	
	 //Part 2 - UNCOMMENT THIS
	for(;;)  //loop forever
	{
	/*	uint16_t dc = 0;
		uint16_t freq = 10000; // Frequency = 10 kHz 
		uint16_t dir = 0;
		char c = 48;
		int i=0;
		delay(10);
		SetServoDuty(5);
		delay(10);
		SetServoDuty(7);
	*/	
		    //Sweep the steering wheel from left to right
 
      //Set position
		

	}  // DC motor
	
	
	/* // Part 3 Stepper Motor
		//  Enable  clocks  on Port D
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	//  Configure  the  Signal  Multiplexer  for  the  Port D GPIO  Pins
	PORTD_PCR0 |= PORT_PCR_MUX(1);
	PORTD_PCR1 |= PORT_PCR_MUX(1);
	PORTD_PCR2 |= PORT_PCR_MUX(1);
	PORTD_PCR3 |= PORT_PCR_MUX(1);
	
	//  Configure  the  GPIO  Pins  for  Output
	GPIOD_PDDR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
	
	while(1){
		// Turn  off  all coils , Set  GPIO  pins to 0
		GPIOD_PCOR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
		//Set  one  pin  high at a time
		if(forward){
			if(phase == 0){GPIOD_PSOR = (1 << 0); phase ++;} //A, 1a
			else if(phase  == 1){GPIOD_PSOR = (1 << 1); phase ++;} //B,2a
			else if (phase  == 2) {GPIOD_PSOR = (1 << 2); phase ++;} //C,1b
			else {GPIOD_PSOR = (1 << 3); phase =0;} //D,2b
		}else    {// reverse
			if (phase == 0) {GPIOD_PSOR = (1 << 3); phase ++;} //D,2b
			else if (phase  == 1) {GPIOD_PSOR = (1 << 2); phase ++;} //C,1b
			else if (phase  == 2) {GPIOD_PSOR = (1 << 1); phase ++;} //B,2a
			else {GPIOD_PSOR = (1 << 0); phase =0;} //A,1a
		}
	
		delay(10);   // smaller  values = faster  speed
	}
	*/
	
	return 0;
}


/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		__ASM("nop");// Do nothing
	}
}
