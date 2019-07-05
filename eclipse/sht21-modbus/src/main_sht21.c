/*
 * main_lcd_i2c.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Renan Augusto Starke
 *      Instituto Federal de Santa Catarina
 */
/*
#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "lcd_i2c.h"
#include "lcd.h"
#include "avr_usart.h"

#define SHT21 0
#include "avr_twi_master.h"
#include "sht21.h"
		
int main(){
	uint8_t i = 0;
	unsigned char msg[4];
	unsigned char msg_rsp[8] = {0,0,0,0};
	
	FILE *lcd_stream = inic_stream();
	FILE *usart_stream = get_usart_stream();
	
	USART_Init(B9600); //UART Debug
	inic_LCD_4bits();

	DDRC=(1<<PC4)|(1<<PC5);
	PORTC=(1<<PC4)|(1<<PC5);
	
	TWI_Master_Initialise();
	//sei();
	msg[0] = 0x80;
	msg[1] = 0xFE;
	TWI_Start_Transceiver_With_Data(msg, 2);
	_delay_ms(100);
	
	//_delay_ms(15);
	//SDA - PC4 , SCL - PC5
	//TWI_Get_Data_From_Transceiver(msg, 3);
	//fprintf(usart_stream, "%d %d", msg[0], msg[1]);
	for(;;) {
		//cmd_LCD(0x80,0);
		msg[0] = 0x80;
		msg[1] = 0xFE;
		TWI_Start_Transceiver_With_Data(msg, 2);
		_delay_ms(100);
		
		msg[0] = 0x80;
		TWI_Start_Transceiver_With_Data(msg, 1);
		msg[0] = 0xE6;
		msg[1] = 0x44;
		TWI_Start_Transceiver_With_Data(msg, 2);

		char buffer1[4];
		//uint16_t temp_value=read_value(temperature_hold_mode);
		//float tc = -46.85 + 175.72 / 65536.0 * temp_value;
		//dtostrf(tc,5,2,buffer1);
		fprintf(usart_stream, "Temperature:");
		fprintf(usart_stream, "%d", buffer1);
		fprintf(usart_stream, " °C\r\n");
		_delay_ms(100);
		//msg_rsp[1] = -46.85 + 175.72*(msg_rsp[1]/0xFFFF);
		//fprintf(usart_stream, "\n\r");
		//fprintf(lcd_stream, "%c %c %c %c", msg_rsp[0], msg_rsp[1], msg_rsp[2], msg_rsp[3]);
		//_delay_ms(500);
	}
}

uint16_t read_value(uint8_t reg){
	char data[4], crc;
	uint16_t result;
	data[0]=SHT21_i2c_write;
	data[1]=reg;
	TWI_Start_Transceiver_With_Data(data, 2);
	data[0] = SHT21_i2c_read;
	TWI_Start_Transceiver_With_Data(data, 1);
	TWI_Get_Data_From_Transceiver(data, 3);

	crc = data[3];
	result=(data[0]<<8) | data[1];
	//checksum(result,4,crc);
	//result &= 0xFFFC;
	return result;
}

*/
//Password: esp-open-rtos
/*E
void TWI_Master_Initialise( void );
unsigned char TWI_Transceiver_Busy( void );
unsigned char TWI_Get_State_Info( void );
void TWI_Start_Transceiver_With_Data( unsigned char * , unsigned char );
void TWI_Start_Transceiver( void );
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *, unsigned char );
*/
