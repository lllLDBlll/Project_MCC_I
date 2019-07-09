/*
 * main.c
 * Temperature Chamber Project
 * Modbus RTU + SHT21
 *
 * Created: 28/06/2019 15:03:32
 *  Author: Leonardo D. Batista
 */
#include <avr/io.h>
/* Cabeçalhos e vetores de interrupções */
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "../lib/avr_timer.h"
#include "../lib/avr_usart.h"
#include "../lib/avr_gpio.h"
#include "../lib/bits.h"
#include "../lib/lcd.h"

/*My includes*/
#include "../lib/modbus_rtu.h"
#include "../lib/sht21.h"
#include "../lib/avr_twi_master.h"

#define F_CPU 16000000UL //Define CPU Clock
#define HIGH 	1
#define LOW 	0

#define MQTT 	1	//test MQTT by ESP01
#define COOLER 	1
#define SHT21 	1

float tc, rh;

#if COOLER
#define DC_PORT GPIO_D
#define DC_PIN	PD3
/*
void timer2_pwm_hardware_init(){
	//fOCnxPWM = (fclk_I/O)/(N*256)
	//PD5: pino OC0B como saída
	GPIO_D->DDR |= SET(PD3);

	// Table 15-6.  Compare Output Mode, Fast PWM Mode
	// COM0B1   COM0B0  Desc:
	//   0       0       Normal port operation, OC0B disconnected.
	//    0       1       Reserved
	//    1       0       Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode)
	//    1       1       Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode).
	//
	// WGM02, WGM01 WGM00 setados: modo PWM rápido com TOP em OCRA
	TIMER_2->TCCRA = SET(WGM01) | SET(WGM00) | SET(COM0B1);
	TIMER_2->TCCRB = SET(WGM02) | SET(CS00);

	// OCRA define frequência do PWM
	TIMER_2->OCRA = 200;

	// OCRB define razão cíclica:  OCRB / OCRA
	TIMER_2->OCRB = 150;
}

inline void set_dutty(uint8_t dutty){
	if (dutty <= TIMER_2->OCRA)
		TIMER_2->OCRB = dutty;
}
 */
#endif

int main(void){
	FILE *lcd_stream;
	lcd_stream = inic_stream();
	/* Inicializa hardware do LCD */
	inic_LCD_4bits(); //Just inic in the main

#if MQTT
	modbus_rtu_init();
#endif

#if SHT21
	sht21_init();
#endif

#if COOLER
	//uint8_t valor_pwm = 1;
	DDRD = SET_BIT(DDRD, DC_PIN); //set as output pin cooler

	/* Configura timer em modo PWM */
	//timer2_pwm_hardware_init();
	//valor_pwm = 199;
	//set_dutty(valor_pwm);
	//sei();
#endif

	cmd_LCD(0x01,0);
	cmd_LCD(0x80,0);
	fprintf(lcd_stream,"%s", "START!");
	_delay_ms(1000);

	while(1){
#if SHT21
		//uint16_t temp_value = sht21_read(T_NO_HOLD); //I2C_buf[1]<<8 | I2C_buf[2]>>2;
		//tc = -46.85 + (175.72 / 65536.0) * temp_value;
		//uint16_t hum_value = sht21_read(RH_NO_HOLD);//I2C_buf[1]<<8 | I2C_buf[2]>>2;
		//rh = -6 + (125.0 / 65536.0) * hum_value;
		cmd_LCD(0x80,0);
		char buffer[4];
		tc = sht21_get_temp();
		dtostrf(tc,5,2,buffer);
		fprintf(lcd_stream, "TC = %s *C", buffer);
		cmd_LCD(0xC0,0);
		rh = sht21_get_hum();
		dtostrf(rh,5,2,buffer);
		fprintf(lcd_stream, "RH = %s %%", buffer);
		_delay_ms(2000);
#endif

#if MQTT
		modbus_rtu_write(ADDR_S_0, tc);
		modbus_rtu_write(ADDR_S_1, rh);
		u16 ret = modbus_rtu_read(ADDR_A_0);

#if COOLER
		if(ret){
			GPIO_SetBit(DC_PORT,DC_PIN);
		}else{
			GPIO_ClrBit(DC_PORT,DC_PIN);
		}
		//valor_pwm = 180;
		//set_dutty(valor_pwm);
		//_delay_ms(1000);
#endif

		cmd_LCD(0x01,0);
		cmd_LCD(0x80,0);
		fprintf(lcd_stream,"ATUADOR_0: %d", ret);
		_delay_ms(2000);
#endif
	}
}
