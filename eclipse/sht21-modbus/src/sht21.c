/*
 * sht21.c
 *
 * Created: 04/12/2018 07:46:45
 *  Author: Leonardo D. Batista
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "../lib/avr_twi_master.h"
#include "../lib/sht21.h"

void sht21_init(){
	uint8_t buff[2];
	/* Inicializa modo líder */
	TWI_Master_Initialise();
	sei();
	buff[0] = SHT21_WRITE;
	buff[1] = SOFT_RESET;
	TWI_Start_Transceiver_With_Data(buff, 2); //Send message to transceiver
	_delay_ms(100);
}

uint16_t sht21_read(uint8_t reg){
	uint8_t buff[4];
	buff[0] = SHT21_WRITE;
	buff[1] = reg;
	TWI_Start_Transceiver_With_Data(buff, 2);

	_delay_ms(100);

	buff[0] = SHT21_READ;
	TWI_Start_Transceiver_With_Data(buff, 4);
	TWI_Get_Data_From_Transceiver(buff, 4);
	return buff[1]<<8 | buff[2]>>2;
}

void sht21_write(unsigned char *msg, unsigned char msgSize){
	TWI_Start_Transceiver_With_Data(msg, msgSize);
}

uint16_t checksum(unsigned char data[], uint8_t byte, uint8_t check){
	uint8_t crc=0;
	uint8_t bytectr,bit;
	for (bytectr=0; bytectr<byte;bytectr++){
		crc^=(data[bytectr]);
		for (bit=8;bit>0;bit--){
			if(crc&0x80){
				crc=(crc<<1)^POLYNOMIAL;
			}
			else{
				crc=crc<<1;
			}
		}
	}
	if (crc!=check){
		return 0;
	}
	else{
		return data;
	}
}

/*
void TWI_Master_Initialise( void );
unsigned char TWI_Transceiver_Busy( void );
unsigned char TWI_Get_State_Info( void );
void TWI_Start_Transceiver_With_Data( unsigned char * , unsigned char );
void TWI_Start_Transceiver( void ); //Call this function to resend the last message.
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *, unsigned char );
 */
