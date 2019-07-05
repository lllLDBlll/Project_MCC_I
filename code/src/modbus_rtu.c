/*
 * modbus.c
 *
 * Created: 14/12/2018 15:04:37
 *  Author: Leonardo D. Batista
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "modbus_rtu.h"
#include "avr_usart.h"
#include "avr_gpio.h"
#include "lcd.h"

#define DEBUG 0

FILE *lcd_stream;
FILE *debug;

typedef struct frame{
	uint8_t addr;
	uint8_t cmd;
	uint8_t reg[2];							//Big Endian
	uint8_t data[2];						//Big Endian
	uint8_t crc[2];							//Big Endian
	char end;	
};

void modbus_rtu_init(){
	inic_LCD_4bits();
	USART_Init(BAUD);
	lcd_stream = inic_stream();
	debug = get_usart_stream();
}

void jump_2nd_line(uint8_t max){
	for(int i = 0; i < max; i++){
		escreve_LCD(" ");
	}
}

uint8_t *modbus_rtu_read(){
	frame_t f;
	char c = "R";
	fprintf(lcd_stream,"%c", c);

	char buff[9];
	
	for(uint8_t i=0; i<LEN; i++){
		buff[i] = USART_rx();		//Receive one byte
	}
	
	//Debug Pisca LED
	PORTB = 1 << PB5;
	_delay_ms(250);
	PORTB = 0 << PB5;
	_delay_ms(250);
		
	fprintf(lcd_stream,"%s", buff);
	//jump_2nd_line();
	//_delay_ms(1000);
	
	f.addr = buff[0];
	f.cmd = buff[1];
	//f.reg = (buff[2] << 8) | buff[3];
	f.reg[0] = buff[2];
	f.reg[1] = buff[3];	
	f.data[0] = buff[4];
	f.data[1] = buff[5];
	//f.crc = (buff[6] << 8) | buff[7];
	f.crc[0] = buff[6];
	f.crc[1] = buff[7];
	f.end = '\0';
	/*
	for(uint8_t *i=&f; *i; i++){
		fprintf(lcd_stream,"%c", i);
		fprintf(debug,"\n%c", 'u');
	}*/
	_delay_ms(10);
	//fprintf(debug,"%s\r", &f);
	
#if DEBUG
	fprintf(debug,"Addr: %c\n\r", f.addr);
	fprintf(debug,"Cmd: %c\n\r", f.cmd);
	fprintf(debug,"Reg: %c", f.reg[0]);
	fprintf(debug,"%c\n\r", f.reg[1]);
	fprintf(debug,"Data: %c", f.data[0]);
	fprintf(debug,"%c\n\r", f.data[1]);
	fprintf(debug,"CRC: %c", f.crc[0]);
	fprintf(debug,"%c\n\r", f.crc[1]);
	nibble_data(f.data);
	fprintf(debug,"Data: %c", f.data[0]);
	fprintf(debug,"%c\n\r", f.data[1]);
	fprintf(debug,"%s\r", buff);
#endif
	//return &f;
}

//<addr><cmd><reg><data><crc>
void modbus_rtu_write(uint16_t reg, uint16_t data){
	uint16_t ret = 0;
	uint8_t tmp[10];
	cmd_LCD(0x80,0);
	//cmd_LCD(0x01,0); //Clear screen
	char *c = "W:";
	fprintf(lcd_stream,"%s", c);
	
	//fprintf(debug,"%s\n\r", &tmp);
	//Monta o pacote
	tmp[0] = ESP_ADDR;
	tmp[1] = W_CMD;
	tmp[2] = reg >> 8;
	tmp[3] = reg;
	tmp[4] = data >> 8;
	tmp[5] = data;
	//uint16_t CRC16_2(uint8_t *buf, int len)
	ret = CRC16_2(tmp,6);
	tmp[6] = ret >> 8;
	tmp[7] = ret;
	
	for(uint8_t i=0; i<LEN; i++){
		fprintf(debug,"%x ", tmp[i]);
		fprintf(lcd_stream,"%X", tmp[i]);
	}
	//jump_2nd_line(0x34);
	jump_2nd_line(0x31);
	modbus_rtu_read();
	//fprintf(debug,"\n\r");
}

void nibble_data(uint8_t *data){
	uint8_t tmp;
	tmp = data[0];
	data[0] = data[1];
	data[1] = tmp;
}

uint16_t CRC16_2(uint8_t *buf, int len){
	uint32_t crc = 0xFFFF;
	int i;
	
	for (i = 0; i < len; i++){
		
		crc ^= (uint16_t)buf[i];			// XOR byte into least sig. byte of crc
		
		for (int i = 8; i != 0; i--) {		// Loop over each bit
			if ((crc & 0x0001) != 0) {		// If the LSB is set
				crc >>= 1;					// Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else // Else LSB is not set
				crc >>= 1; // Just shift right
		}
	}
	//fprintf(debug,"%d - %c\n\r", crc, crc);
	return crc;
}