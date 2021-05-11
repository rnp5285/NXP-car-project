/*
 * File:        uart3.c
 * Purpose:     Provide UART routines for serial IO
 * Name:				Shery George Mathews
 * Notes:		
 *
 */

#ifndef UART_H
#define UART_H
#include "MK64F12.h"
#include "uart.h"
#endif

#include "MK64F12.h"
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

void uart0_put(char *ptr_str);
void uart0_init(void);
uint8_t uart0_getchar(void);
void uart0_putchar(char ch);

void uart3_put(char *ptr_str);
void uart3_init(void);
uint8_t uart3_getchar(void);
void uart3_putchar(char ch);

void LED_Init(void);

void uart0_init()
{
//define variables for baud rate and baud rate fine adjust
uint16_t ubd, brfa;

//Enable clock for UART
SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
 

//Configure the port control register to alternative 3 (which is UART mode for K64)
	PORTB_PCR16 |= PORT_PCR_MUX(3);
	PORTB_PCR17 |= PORT_PCR_MUX(3);


/*Configure the UART for establishing serial communication*/

//Disable transmitter and receiver until proper settings are chosen for the UART module
UART0_C2 &= ~(UART_C2_RE_MASK);
UART0_C2 &= ~(UART_C2_TE_MASK);

//Select default transmission/reception settings for serial communication of UART by clearing the control register 1
UART0_C1 = 0x00;

//UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
//13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
//BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
//BRFA is defined by the lower 4 bits of control register, UART0_C4 

//calculate baud rate settings: ubd = UART module clock/16* baud rate
ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

//clear SBR bits of BDH
UART0_BDH = 0000;  

//distribute this ubd in BDH and BDL
UART0_BDL = ubd;
UART0_BDH = (ubd >>8);

//BRFD = (1/32)*BRFA 
//make the baud rate closer to the desired value by using BRFA
brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

//write the value of brfa in UART0_C4
UART0_C4 = brfa;
	
//Enable transmitter and receiver of UART
 UART0_C2 |= (1<< UART_C2_RE_SHIFT);
 UART0_C2 |= (1<< UART_C2_TE_SHIFT);
 
}




void uart3_init()
{
//define variables for baud rate and baud rate fine adjust
uint16_t ubd, brfa;

//Enable clock for UART
SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
 

//Configure the port control register to alternative 3 (which is UART mode for K64)
	PORTB_PCR10 |= PORT_PCR_MUX(3);
	PORTB_PCR11 |= PORT_PCR_MUX(3);


/*Configure the UART for establishing serial communication*/

//Disable transmitter and receiver until proper settings are chosen for the UART module
UART3_C2 &= ~(UART_C2_RE_MASK);
UART3_C2 &= ~(UART_C2_TE_MASK);

//Select default transmission/reception settings for serial communication of UART by clearing the control register 1
UART3_C1 = 0x00;

//UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
//13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
//BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
//BRFA is defined by the lower 4 bits of control register, UART0_C4 

//calculate baud rate settings: ubd = UART module clock/16* baud rate
ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

//clear SBR bits of BDH
UART3_BDH = 0000;  

//distribute this ubd in BDH and BDL
UART3_BDL = ubd;
UART3_BDH = (ubd >>8);

//BRFD = (1/32)*BRFA 
//make the baud rate closer to the desired value by using BRFA
brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

//write the value of brfa in UART3_C4
UART3_C4 = brfa;
	
//Enable transmitter and receiver of UART
 UART3_C2 |= (1<< UART_C2_RE_SHIFT);
 UART3_C2 |= (1<< UART_C2_TE_SHIFT);
}





void LED_Init(void){
	// Enable clocks on Ports B and E for LED timing
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //PTB
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; //PTE
	
	// Configure the Signal Multiplexer for GPIO
  PORTB_PCR22 = PORT_PCR_MUX(1);	//PTB22
	PORTE_PCR26 = PORT_PCR_MUX(1);	//PTE26
	PORTB_PCR21 = PORT_PCR_MUX(1);	//PTB21
	
	// Switch the GPIO pins to output mode
	GPIOB_PDDR |= (1<<22);
	GPIOE_PDDR |= (1<<26);
	GPIOB_PDDR |= (1<<21);
	
	// Turn off the LEDs
  GPIOB_PDOR |= (1<<22);
	GPIOE_PDOR |= (1<<26);
	GPIOB_PDOR |= (1<<21);
}





uint8_t uart0_getchar()
{
/* Wait until there is space for more data in the receiver buffer*/
while(1){
	if(UART0_S1 & UART_S1_RDRF_MASK) {
		
	/* Return the 8-bit data from the receiver */
	return UART0_D;
	}
}
}


uint8_t uart3_getchar()
{
/* Wait until there is space for more data in the receiver buffer*/
while(1){
	if(UART3_S1 & UART_S1_RDRF_MASK) {
	/* Return the 8-bit data from the receiver */
	return UART3_D;
	}
}
}



void uart0_putchar(char ch)
{
/* Wait until transmission of previous bit is complete */
while(1){
	if(UART0_S1 & UART_S1_TDRE_MASK) {
/* Send the character */
UART0_D = ch;
		return;
}
	}
}
	

void uart3_putchar(char ch)
{
/* Wait until transmission of previous bit is complete */
while(1){
	if(UART3_S1 & UART_S1_TDRE_MASK) {
/* Send the character */
UART3_D = ch;
		return;
}
	}
}



void uart0_put(char *ptr_str){
	/*use putchar to print string*/
	while(*ptr_str)
		uart0_putchar(*ptr_str++);
}

void uart3_put(char *ptr_str){
	/*use putchar to print string*/
while(*ptr_str)
		uart3_putchar(*ptr_str++);
}
