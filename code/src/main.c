/*
 * ModbusRTU_LCD.c
 *
 * Created: 14/12/2018 15:03:32
 *  Author: Leonardo D. Batista
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
/* Cabeçalhos e vetores de interrupções */
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "modbus_rtu.h"
#include "avr_usart.h"
#include "avr_gpio.h"
#include "lcd.h"

void write_main(){
	char *c = "The God is Good";
	escreve_LCD(c);
	_delay_ms(500);
	//cmd_LCD(0x01, 0);
	jump_2nd_line();
	c = "Although...";
	escreve_LCD(c);
	_delay_ms(500);
	cmd_LCD(0x01, 0);
	c = "Renan is Better";
	escreve_LCD(c);
	_delay_ms(500);
	cmd_LCD(0x01, 0);
	_delay_ms(250);
}

int main(void){
	uint8_t x = 0;
	DDRB = 0xFF;
	PORTB = 0x00;
	//modbus_rtu_init();
	inic_LCD_4bits();
	USART_Init(B9600);
    while(1){
		//modbus_rtu_write(ADDR_S_0, 0x0400);
		//cmd_LCD(0x01,0);
		//inic_LCD_4bits();
		cmd_LCD(0x0D, 0);
		write_main();
		USART_tx('P');
		switch(x){
			case ADDR_A_0:
				PORTB = 1 << PB0;
			break;
			case ADDR_A_1:
				PORTB = 1 << PB1;
			break;
			case ADDR_A_2:
				PORTB = 1 << PB2;
			break;
			case ADDR_A_3:
				PORTB = 1 << PB3;
			break;
		}
    }
}
