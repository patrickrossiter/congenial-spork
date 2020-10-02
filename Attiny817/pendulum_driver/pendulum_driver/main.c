/*
https://www.robot-electronics.co.uk/i2c-tutorial
https://www.avrfreaks.net/comment/2294081
https://community.nxp.com/docs/DOC-98836
*/

#include <atmel_start.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pendulum.h"
// #include "pid.c"
//
#define PWMFREQ			.5e3
#define TCA0_PERIOD		(F_CPU / (2*4*PWMFREQ)) - 1;				// See equation in 20.3.3.4.2 of manual

#define F_TCA			(F_CPU / 8)
#define F_CALC			100			// Hz
#define F_TWI			400e3		// Hz (not working)

#define TOP_SPEED		0xFFF
#define MMA8451_ADDR	0x1D
#define	TWI_WRITE		0x0
#define	TWI_READ		0x1
//
#define STATUS_REG		0x00
#define OUT_X_MSB		0x01
#define OUT_X_LSB		0x02
#define OUT_Y_MSB		0x03
#define OUT_Y_LSB		0x04
#define OUT_Z_MSB		0x05
#define OUT_Z_LSB		0x06
#define SYSMOD			0x0B
#define XYZ_DATA_CFG	0x0E	

#define CTRL_REG1		0x2A
#define CTRL_REG2		0x2B
#define CTRL_REG3		0x2C
#define CTRL_REG4		0x2D
#define CTRL_REG5		0x2E

#define OFF_X_REG		0x2F
#define OFF_Y_REG		0x30
#define OFF_Z_REG		0x31

#define SENSITIVITY_2G	4096		// Counts per g
#define SENSITIVITY_4G	2048		// Counts per g
#define SENSITIVITY_8G	1024		// Counts per g
//
#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)
//
void TCA_setup(void);
void TCB_setup(void);
void I2C_setup(void);
void ADC_setup(void);
//
uint8_t I2C_ReadRegister(uint8_t device_address, uint8_t register_address);
void I2C_ReadMultiRegisters(uint8_t device_address, uint8_t start_register_address,
		uint8_t registers, int16_t output_array[]);
void USART_write_char(char USART_character);
void USART_setup(long baud);
void USART_multi_char(uint16_t *input, int length);
uint32_t get_rotor_position(void);
void get_speed_command(void);
//
uint32_t speed_command, adc_result;
int32_t direction, prev_dir;
struct PID_DATA pidData;

//
volatile int loop;
int32_t position_error;
int32_t position_setpoint;
int32_t current_position;
int32_t loop_counter = 0;
uint32_t output_data[6];
uint32_t rotorAngle, max_angle;
char usart_send_buffer[4];
int32_t usart_send_length;
int32_t Vt = 10200;
int32_t P = 800;
int32_t Q = 200;
int32_t usart_idx;
uint16_t *usart_ptr;
int var1 ;

int16_t debug[200];
/*
USART
TXD - PA1
RXD - PA2
XCK - PA3
XDIR - PA4

Button = PC5
LED = PC0
QT button 1 = PA6
QT button 2 = PA7

TCA0 WO = PB0
Direction = PB1
PB2 not working?
Timing indicator = PB3
*/
int main(void) {
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	// 
	PORTB.DIRSET |= (PIN5_bm | PIN3_bm | PIN1_bm | PIN0_bm);
	// 
	// Setup buttons as inputs to change position set point
	// Timer 1 to frequency generation mode
	TCA_setup();	
	// Timer 2 to trigger position calculation every x milliseconds
	TCB_setup();
	// ADC for angle position detection
	ADC_setup();
	sei();
	speed_command = 4500;
	int rotorReference = 194;
	int deadband = 2;
	while (1) {
		if (loop) {
			PORTB.OUTSET = PIN5_bm;
			
			rotorAngle = get_rotor_position();
			// int32_t pidOutput = pid_Controller(rotorReference, rotorAngle, &pidData);
			// Simple bang-bang stabiliser for non-inverted pendulum
			if (rotorAngle > (rotorReference+deadband)){
				direction = 1;
				speed_command = 4900;
			}
			else if (rotorAngle < (rotorReference-deadband)){
				direction = 0;
				speed_command = 4900;				
			}
			else {
				speed_command = 0;
			}
			loop_counter++;
			
			PORTB.OUTCLR = PIN5_bm;
			loop = 0;
		}
	}
}
//
void USART_write_char(char USART_character) {
	while(!(USART0.STATUS & USART_DREIF_bm));
	USART0.TXDATAL = USART_character;
}
//
void USART_multi_char(uint16_t *input, int length) {
	usart_send_length = length;
	
	usart_idx = 0;
	usart_ptr = input;
// 	volatile char test = *(usart_ptr+2);
	
	while(!(USART0.STATUS & USART_DREIF_bm));
	USART0.TXDATAL = usart_send_buffer[usart_idx];
	USART0.CTRLA |= USART_DREIE_bm;
}
//
uint32_t get_rotor_position(void) {
	// Read rotor position from ADC channel 7 (PA7)
	ADC0.COMMAND = ADC_STCONV_bm;
	while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));
	//
	uint32_t angle = (uint32_t)ADC0.RES * 360 / 1017;
	if (ADC0.RES > max_angle) max_angle = ADC0.RES;
	return angle;
}
//
void get_speed_command(void) {
	// Read rotor position from ADC channel 7 (PA07)
	ADC0.COMMAND = ADC_STCONV_bm;
	while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));
	//
	adc_result = (uint16_t)ADC0.RES;
	if (adc_result >= 512) {
 		direction = 1;
		speed_command = (adc_result - 512) * 1250 / 512;
	}
	else {
		direction = 0;
		speed_command = (512 - adc_result) * 1250 / 512;		
	}
}
//
void ADC_setup(void) {
	// Hall sensor using AIN7 - PA07
	// 360/1024 = 45/128
	//
	// DAC setup for testing - PA6 and PA7 shorted
	VREF.CTRLA |= VREF_DAC0REFSEL_4V34_gc;
	VREF.CTRLB |= VREF_DAC0REFEN_bm;
	DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;
	DAC0.DATA = 0xFF;
	// 
	PORTA.DIRCLR = PIN7_bm;
	//
	VREF.CTRLA |= VREF_ADC0REFSEL_4V34_gc;
	VREF.CTRLB |= VREF_ADC0REFEN_bm;
	//
	ADC0.CTRLA |= ADC_RESSEL_10BIT_gc;
	ADC0.CTRLA |= ADC_FREERUN_bm;
	ADC0.CTRLC |= ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm | ADC_PRESC_DIV16_gc;	// 20 MHz / 16 = 1.25 MHz ADC clock
	ADC0.CALIB = ADC_DUTYCYC_DUTY25_gc;
	ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;
	ADC0.CTRLA |= ADC_ENABLE_bm;
		
// 	for (int i = 0; i < 128; i++){
// 		DAC0.DATA = i*2;
// 		//
// 		ADC0.COMMAND = ADC_STCONV_bm;
// 		while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));
// 	
// 		result[i] = ADC0.RES >> 2;
// 	}	
// 	result[255] = 0;
	
}
//
void USART_setup(long baud) {
	/*
	RXD - PB3
	TXD - PB2
	XCK - PB1
	XDIR - PB0
	*/
// 	PORTMUX.CTRLB |= PORTMUX_USART0_ALTERNATE_gc;
	// For interrupt-driven USART operation, global interrupts should be disabled during the initialization.
	cli();
	// 24.3.1 Initialization
	// For setting the USART in full-duplex mode, the following initialization sequence is recommended:
	// 1. Set the TxD pin value high, and optionally set the XCK pin low (OUT[n] in PORT.OUT).
	PORTB.OUTSET = (PIN5_bm | PIN2_bm | PIN1_bm);		// TXD
	PORTB.DIRSET = (PIN5_bm | PIN2_bm | PIN1_bm);		// TXD
	// 2. Set the TxD and optionally the XCK pin as output (DIR[n] in PORT.DIR).
	PORTB.DIR |= PIN2_bm;		// TXD
	PORTB.DIR |= PIN1_bm;		// XCK
	// 3. Set the baud rate (USART.BAUD) and frame format.
	USART0.BAUD = ((F_CPU) / (2 * baud)) << 6;		// S = 2 even though this is async mode (determined by experimentation)
	// 4. Set the mode of operation (enables XCK pin output in synchronous mode).
	#define USART_PMODE2_bm  (2<<5)  /* Parity Mode bit 1 mask. */
	USART0.CTRLC = USART_PMODE2_bm | USART_CHSIZE_8BIT_gc;
	// 5. Enable the transmitter or the receiver, depending on the usage.
	USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	sei();
}
//
void I2C_WriteRegister(uint8_t device_address, uint8_t register_address, 
		uint8_t write_value){

	TWI0.MCTRLA = TWI_ENABLE_bm;
	TWI0.MSTATUS |= TWI_BUSSTATE1_bm;

	TWI0.MADDR = (device_address << 1) | (TWI_WRITE);
	while(!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MSTATUS |= TWI_WIF_bm;

	TWI0.MDATA = register_address;
	while(!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MSTATUS |= TWI_WIF_bm;
	// 
	TWI0.MDATA = write_value;
	while(!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MSTATUS |= TWI_WIF_bm;

	TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}
//
uint8_t I2C_ReadRegister(uint8_t device_address, uint8_t register_address) {
	uint8_t read_value;
	
	TWI0.MCTRLA = TWI_ENABLE_bm;
	TWI0.MSTATUS |= TWI_BUSSTATE1_bm;
	
	TWI0.MADDR = (device_address << 1) | (TWI_WRITE);
	while(!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MSTATUS |= TWI_WIF_bm;

	TWI0.MDATA = register_address;
	while(!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MSTATUS |= TWI_WIF_bm;

	TWI0.MCTRLB |= TWI_MCMD_REPSTART_gc;

	TWI0.MCTRLB |= TWI_ACKACT_NACK_gc;
	TWI0.MADDR = (device_address << 1) | (TWI_READ);
	while(!(TWI0.MSTATUS & TWI_RIF_bm));
	TWI0.MSTATUS |= TWI_RIF_bm;
	TWI0.MCTRLB |= TWI_ACKACT_ACK_gc;
				
	read_value = TWI0.MDATA;

	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
	
	return read_value;
}
//
void I2C_ReadMultiRegisters(uint8_t device_address, uint8_t start_register_address,
								uint8_t registers, int16_t output_array[]) {
	uint8_t read_value;
		
	TWI0.MCTRLA = TWI_ENABLE_bm;
	TWI0.MSTATUS |= TWI_BUSSTATE1_bm;
	
	TWI0.MADDR = (device_address << 1) | (TWI_WRITE);
	while(!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MSTATUS |= TWI_WIF_bm;

	TWI0.MDATA = start_register_address;
	while(!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MSTATUS |= TWI_WIF_bm;

	TWI0.MCTRLB |= TWI_MCMD_REPSTART_gc;
	TWI0.MCTRLB |= TWI_ACKACT_ACK_gc;
	
	TWI0.MADDR = (device_address << 1) | (TWI_READ);
	while(!(TWI0.MSTATUS & TWI_RIF_bm));
	TWI0.MSTATUS |= TWI_RIF_bm;

	for (int i=0; i<registers; i++)	{
		read_value = TWI0.MDATA;
		output_array[i] = read_value;
		TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
		while(!(TWI0.MSTATUS & TWI_RIF_bm));
		TWI0.MSTATUS |= TWI_RIF_bm;		
	}
	TWI0.MCTRLB |= TWI_ACKACT_NACK_gc;
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
	TWI0.MCTRLB |= TWI_ACKACT_ACK_gc;
}
//
void I2C_setup(void) {
	PORTA.DIRSET |= (PIN2_bm | PIN1_bm);
	PORTMUX.CTRLB |= PORTMUX_TWI0_ALTERNATE_gc;
	
	TWI0.MSTATUS |= TWI_BUSSTATE1_bm;
	TWI0.MBAUD = (F_CPU/(F_TWI-(F_CPU*300e-9)-10))/2;
	TWI0.MBAUD = 5;
	
	TWI0.MCTRLA |= (TWI_ENABLE_bm);
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	//
	I2C_WriteRegister(MMA8451_ADDR, CTRL_REG2, 0x40);
	I2C_WriteRegister(MMA8451_ADDR, XYZ_DATA_CFG, 0x00);			// +/-2g range -> 1g = 16384/4 = 4096 counts
	I2C_WriteRegister(MMA8451_ADDR, CTRL_REG2, 0x02);				// High Resolution mode
	I2C_WriteRegister(MMA8451_ADDR, CTRL_REG1, 0x3D);				// ODR = 1.56Hz, Reduced noise, Active mode
	//
    I2C_WriteRegister(MMA8451_ADDR, CTRL_REG3, 0x00);				// Push-pull, active low interrupt
    I2C_WriteRegister(MMA8451_ADDR, CTRL_REG4, 0x01);				// Enable DRDY interrupt
    I2C_WriteRegister(MMA8451_ADDR, CTRL_REG5, 0x01);				// DRDY interrupt routed to INT1 - PTA14
    I2C_WriteRegister(MMA8451_ADDR, CTRL_REG1, 0x0D);				// ODR = 400 Hz, Reduced noise, Active mode 
}
//

void TCA_setup(void) {
	TCA0.SINGLE.CTRLA |= (TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV4_gc);
	TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_FRQ_gc | TCA_SINGLE_CMP0EN_bm;
	TCA0.SINGLE.PER = TCA0_PERIOD;
	TCA0.SINGLE.INTCTRL |= TCA_SINGLE_CMP0_bm;
}
//
void TCB_setup(void) {
	TCB0.CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_CLKTCA_gc;
	TCB0.INTCTRL |= TCB_CAPT_bm;
	TCB0.CCMP = F_TCA / F_CALC;
}
//
ISR(TCA0_CMP0_vect) {
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;

	TCA0.SINGLE.CMP0 = speed_command;
	if (direction != prev_dir) {
		// Stop PWM
		TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
		// Change PB01 to value of direction
		if (direction) PORTB.OUTSET = PIN1_bm;
		else PORTB.OUTCLR = PIN1_bm;
		// Restart PWM
		TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
	}
	prev_dir = direction;
}
//
ISR(TCB0_INT_vect) {
	TCB0.INTFLAGS = TCB_CAPT_bm;
	loop = 1;
}
//
ISR(USART0_DRE_vect) {
	PORTB.OUTSET = PIN5_bm;

	usart_idx++;
	if (usart_idx == usart_send_length) USART0.CTRLA &= ~USART_DREIE_bm;
	while(!(USART0.STATUS & USART_DREIF_bm));
	USART0.TXDATAL = usart_send_buffer[usart_idx];

	PORTB.OUTCLR = PIN5_bm;
}
