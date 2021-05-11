/*
 * Freescale Cup linescan camera code
 *
 *	This method of capturing data from the line
 *	scan cameras uses a flex timer module, periodic
 *	interrupt timer, an ADC, and some GPIOs.
 *	CLK and SI are driven with GPIO because the FTM2
 *	module used doesn't have any output pins on the
 * 	development board. The PIT timer is used to 
 *  control the integration period. When it overflows
 * 	it enables interrupts from the FTM2 module and then
 *	the FTM2 and ADC are active for 128 clock cycles to
 *	generate the camera signals and read the camera 
 *  output.
 *
 *	PTB8			- camera CLK
 *	PTB23 		- camera SI
 *  ADC0_DP1 	- camera AOut
 *
 * Author:  Alex Avery
 * Created:  11/20/15
 * Modified:  11/23/15
 */

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"
#include "pwm.h"
#include <math.h>

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);
void delay(float del);

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];

// These variables are for streaming the camera
//	 data over UART
int debugcamdata = 1;
int capcnt = 0;
char str[100];
int stopCar = 0;
// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

int main(void)
{
	int smooth[122];
	int i;
	int midpoint, middle;
	int stop = 10;
	int left, right;
	int up = 70, down = 18;
	int motorspeed_l_s = 40;
	int motorspeed_r_s = 40;
	float servopwm, servopwmold =0.0;
	float dcpwm, dcpwmold=0.0;
	float CTR = 6.2;
	int counter=0;
	int CTRthresh = 15;
	float kp = 0.042, ki = 0.02, kd= 0.03;
	float error = 0, errorold1=0, errorold2 =0;
	
	
	uart0_init();
	uart3_init();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
	InitDCMotPWM();
	InitServoPWM();

	
	SetServoDuty(CTR); // set servo position to center
	for(;;) {
		//SetDCMotDuty(0,0); // <- DEBUGGING
		
		// braking system
		if(counter>125){
			SetDCMotDuty(-100,-100); 
			delay(1.7);
			counter = 0;
			}
		
		midpoint = 0;
		middle = 0;
		right = 0;
		left = 0;
			
		if (1) { 
			if (capcnt >= 50) {
			midpoint = 0;
			right = 0;
			left = 0;
			GPIOB_PCOR |= (1 << 22);
				
			for (i=4; i<124; i++){
				smooth[i-4] = ((line[i-4]+line[i-3]+line[i-2]+line[i-1]+line[i]+line[i+1]+line[i+2]+line[i+3]+line[i+3])/9000);
			}
			
			
			for (i = 0; i < 127; i++) {
				//.sprintf(str,"%i\n\r", line[i]); // DEBUGGING
				midpoint += (int)smooth[i];
				if(i>= 15 && i<=45) {
					left += (int) smooth[i];
				}
				if(i>=74 && i<=104) {
					right += (int) smooth[i];
				}
		
					middle = (int) smooth[62];
				
				//.uart0_put(str); // DEBUGGING
			}
			midpoint /= 128;
			right /= 30;
			left /= 30;
			stopCar=0;
			error = abs(left-right);
		 
		if (stopCar == 0) {
			// carpet detection
			if ((right < stop) && (left < stop) && (midpoint < stop)) 
				{ 
					SetDCMotDuty(0,0);
					stopCar = 1;
				}
			// turn
			else {	
				error = abs(left-right);
				servopwm = kp*(error
				); //+ ki*((error+errorold1)/2) + kd*(error - 2*errorold1 + errorold2);
				sprintf(str,"%f\n\r",servopwm);
				uart0_put(str);
				
				dcpwm = kp*(error); //+ ki*((error+errorold1)/2) + kd*(error - 2*errorold1 + errorold2);
				
				if (servopwm>.89) {
					// race day, no PID, slow down on turns
					if(right < left){
						SetDCMotDuty(-60,-60);	
						servopwm = 1;
						SetServoDuty(7.5);
						//SetDCMotDuty(motorspeed_l_s-dcpwm-(motorspeed_l_s/4),motorspeed_r_s-(motorspeed_r_s/6));
						SetDCMotDuty(down,up);
					}
					else { 
						SetDCMotDuty(-60,-60);	
						servopwm = 1;
						SetServoDuty(5.3);
						//SetDCMotDuty(motorspeed_l_s-(motorspeed_l_s/6),motorspeed_r_s-dcpwm-(motorspeed_r_s/4));
						SetDCMotDuty(up,down);
					}
				}
				// PID turns
				else {
					counter++;
					if(right < left){ // && (middle > 40)){
						SetServoDuty(CTR+servopwm);
						SetDCMotDuty(motorspeed_l_s,motorspeed_r_s+dcpwm);
					}
					else if((right > left)){ // && (middle > 40)) {
						SetServoDuty(CTR-servopwm);
						SetDCMotDuty(motorspeed_l_s+dcpwm,motorspeed_r_s);
					} 
				}
				dcpwmold = dcpwm;
				servopwmold = servopwm;
				errorold2  = errorold1;
				errorold1 = error;
			 } // else turn
				GPIOB_PSOR |= (1 << 22);
			}  // else not white middle	
			}
		}
	} //for
} //main


/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL = ADC0_RA;
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  FTM2_SC &= ~(FTM_SC_TOF_MASK);
	
	// Toggle clk
	clkval = !clkval;
	GPIOB_PTOR |= (1 << 9); // CLK = 0
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~(FTM_SC_TOIE_MASK);
	
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	//PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //active high clear
	PIT_TFLG0 = PIT_TFLG0;
	// Setting mod resets the FTM counter
	FTM2_MOD = FTM2_MOD;
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	
	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT |= FTM_CNT_COUNT(0);
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN |= FTM_CNTIN_INIT(0);
	
	// Set the period (~10us)
	FTM2_MOD = ((DEFAULT_SYSTEM_CLOCK/10000)/10);
	
	// 50% duty
	FTM2_MOD /= 2; //((DEFAULT_SYSTEM_CLOCK/10000)/2);
	
	// Set edge-aligned mode
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	FTM2_C0SC &= ~(FTM_CnSC_ELSA_MASK);
	
	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~(FTM_SC_TOIE_MASK);
	
	// No prescalar, system clock
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
	
	//enable interrupt
	FTM2_SC &= ~FTM_SC_TOIE_MASK; // disable 
	
	// Set up interrupt
	NVIC_EnableIRQ(FTM2_IRQn);
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR &= ~(PIT_MCR_FRZ_MASK); // In case you need to debug
	
	//Enable Timer
	PIT_MCR &= ~(PIT_MCR_MDIS_MASK);
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = (uint32_t) (DEFAULT_SYSTEM_CLOCK*INTEGRATION_TIME);
	
	// Enable timer interrupts
	PIT_TCTRL0 = PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 = PIT_TFLG0;// PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}


/* Set up pins for GPIO
* 	PTB9 		- camera clk - done
*		PTB23		- camera SI - done
*		PTB22		- red LED -done
*/
void init_GPIO(void){
	// Enable LED and GPIO so we can see results
	// Enable clocks on Ports B and E for LED timing
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //port b clock
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; //port e clock

	// Switch the GPIO pins to output mode
	PORTB_PCR22 = PORT_PCR_MUX(1); //wake up red
	PORTB_PCR21 = PORT_PCR_MUX(1); //blue
	PORTE_PCR26 = PORT_PCR_MUX(1); //green
	
	PORTB_PCR9 = PORT_PCR_MUX(1); //camera clk
	PORTB_PCR23 = PORT_PCR_MUX(1); //camera SI
	
	GPIOB_PDDR = (1 << 21) | (1 << 22); //set GPIO B pin 21, 22 high in PDDR
	GPIOE_PDDR = (1 << 26); //set GPIO E pin 26 high in PDDR
	GPIOB_PDDR = (1 << 9) | (1 << 23);
	
	// Turn off the LEDs
	GPIOB_PDOR = (1 << 21) | (1 << 22); //turn off GPIO B pin 21, 22
	GPIOE_PDOR = (1 << 26); //turn off GPIO E pin 26
	
	GPIOB_PDOR |= (1<<9) | (1<<23);
	return;
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	
		// Single ended 16 bit conversion, no clock divider
		ADC0_CFG1 &= ~ADC_CFG1_ADIV(0);
		ADC0_CFG1 |= ADC_CFG1_MODE(3);
  
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; 
		calib += ADC0_CLP1; 
		calib += ADC0_CLP2;
    calib += ADC0_CLP3; 
		calib += ADC0_CLP4; 
		calib += ADC0_CLPS;
    calib = calib >> 1; 
		calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
    
		ADC0_SC1A &= ~(0x1f << ADC_SC1_ADCH_SHIFT);
    // Set to single ended mode	
		ADC0_SC1A &= ~(ADC_SC1_DIFF_MASK);
		 
	
		// Set up FTM2 trigger on ADC0
		SIM_SOPT7 &= ~SIM_SOPT7_ADC0TRGSEL(0xf); //clear before setting
		SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(10); //select FTM2 as trigger
		SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
		SIM_SOPT7 &= ~(SIM_SOPT7_ADC0PRETRGSEL_MASK); // Pretrigger A
	  
		//enable adc interrupt
		ADC0_SC1A |= ADC_SC1_AIEN_MASK;
		
		// Enable NVIC interrupt
    NVIC_EnableIRQ(ADC0_IRQn);
}


/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(float del){
	int i;
	for (i=0; i<del*50000; i++){
		__ASM("nop");// Do nothing
	}
}