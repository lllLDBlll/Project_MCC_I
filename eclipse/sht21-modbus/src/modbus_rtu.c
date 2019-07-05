/*
 * modbus_rtu.c
 *
 * Created: 05/07/2019 02:18:53
 *  Author: Leonardo D. Batista
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "../lib/modbus_rtu.h"
#include "../lib/avr_usart.h"
#include "../lib/avr_gpio.h"
#include "../lib/lcd.h"

#define DEBUG 1

typedef struct frame{
	u8 addr;
	u8 cmd;
	u8 reg[2];							//Big Endian
	u8 data[2];							//Big Endian
	u8 crc[2];							//Big Endian
};

void modbus_rtu_init(){
	/* Inicializa hardware da USART */
	USART_Init(BAUD);
	lcd_stream = inic_stream();
	/* Inicializa hardware do LCD */
	inic_LCD_4bits();
	fprintf(lcd_stream,"%s", "MODBUS RTU INIT!");
	_delay_ms(1000);
}

//Just One Register not Multiply
u16 modbus_rtu_read(u16 reg_dest){
	u8 pkg[8], rx_pkg[16], i;
	u16 crc;

	pkg[0] = ESP_ADDR;
	pkg[1] = R_CMD;
	pkg[2] = reg_dest >> 8;
	pkg[3] = reg_dest & 0xff;
	pkg[4] = pkg[2];
	pkg[5] = pkg[3];

	crc = CRC16_2(pkg,6);

	pkg[6] = crc >> 8;
	pkg[7] = crc & 0xff;

	for (i=0; i < 8; i++)
		USART_tx(pkg[i]);

	for (i=0; i < 8;i++)
		rx_pkg[i] = USART_rx();

	//checkError();
	for(uint8_t i=0; i<8; i++){
		if(pkg[i] != rx_pkg[i]);
		//Mamamia
	}
#if DEBUG
	cmd_LCD(0x01,0); //Clear display screen
	for(uint8_t i=0; i<6; i++){
		fprintf(lcd_stream,"%x ", rx_pkg[i]);
	}
	_delay_ms(2000);
#endif
	return (rx_pkg[4]<<8 | rx_pkg[5]);
}


//<addr><cmd><reg><data><crc>
void modbus_rtu_write(u16 reg, u16 data){
	u8 pkg[8], rx_pkg[8], i;
	u16 crc;

	//Monta o pacote
	pkg[0] = ESP_ADDR;
	pkg[1] = W_CMD;
	pkg[2] = reg >> 8;
	pkg[3] = reg & 0xff;
	pkg[4] = data >> 8;
	pkg[5] = data & 0xff;

	crc = CRC16_2(pkg,6);

	pkg[6] = crc >> 8;
	pkg[7] = crc & 0xff;

	for (i=0; i < 8; i++)
		USART_tx(pkg[i]);

	for (i=0; i < 8;i++)
		rx_pkg[i] = USART_rx(); //Resposta deve ser Igual ao Enviado

	//checkError();
	for(uint8_t i=0; i<8; i++){
		if(pkg[i] != rx_pkg[i]);
		//Mamamia
	}
#if DEBUG
	cmd_LCD(0x01,0); //Clear display screen
	for(uint8_t i=0; i<6; i++){
		fprintf(lcd_stream,"%x ", rx_pkg[i]);
	}
	_delay_ms(2000);
#endif
}

void nibble_data(uint8_t *data){
	uint8_t tmp;
	tmp = data[0];
	data[0] = data[1];
	data[1] = tmp;
}

uint16_t CRC16_2(uint8_t *buf, int len)
{
	uint32_t crc = 0xFFFF;
	int i;

	for (i = 0; i < len; i++)
	{
		crc ^= (uint16_t)buf[i];          // XOR byte into least sig. byte of crc

		for (int i = 8; i != 0; i--) {    // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                            // Else LSB is not set
				crc >>= 1;                    // Just shift right
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	return crc;
}

