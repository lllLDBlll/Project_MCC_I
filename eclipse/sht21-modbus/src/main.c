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
#define HIGH 1
#define LOW 0

#define MQTT 1	//test MQTT by ESP01
#define COOLER 0
#define SHT21 1

uint8_t I2C_buf[10]={0};
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
	uint16_t data = 0;
	uint8_t buff[2];
	modbus_rtu_init();
	sht21_init();
	cmd_LCD(0x01,0);
	cmd_LCD(0x80,0);
	fprintf(lcd_stream,"%s", "START!");
	_delay_ms(1000);
#endif

#if SHT21
	/* Obtem o stream de depuração */
	FILE *debug = get_usart_stream();

	/* Inicializa hardware da USART */
	USART_Init(B9600);

	/* Mensagem incial: terminal do Proteus
	 * utiliza final de linha com '\r' */

	sht21_init();
#endif

#if COOLER
	uint8_t valor_pwm = 1;
	DDRD = SET(DC_PIN); //set as output pin cooler

	/* Configura timer em modo PWM */
	//timer2_pwm_hardware_init();
	//valor_pwm = 199;
	//set_dutty(valor_pwm);
	//sei();
#endif

	while(1){
#if SHT21
		uint16_t temp_value = sht21_read(T_NO_HOLD); //I2C_buf[1]<<8 | I2C_buf[2]>>2;
		tc = -46.85 + (175.72 / 65536.0) * temp_value;
		char buffer1[4];
		dtostrf(tc,5,2,buffer1);
		//fprintf(debug, "TC = %s *C \n\r", buffer1);

		char buffer2[4];
		uint16_t hum_value = sht21_read(RH_NO_HOLD);//I2C_buf[1]<<8 | I2C_buf[2]>>2;
		rh = -6 + (125.0 / 65536.0) * hum_value;
		dtostrf(rh,5,2,buffer2);
		//fprintf(debug, "RH = %s %%  \n\r", buffer2);

		//fprintf(debug, "%s\n\r","----------");
		_delay_ms(1000);
#endif

#if MQTT
		modbus_rtu_write(ADDR_S_0, tc);
		modbus_rtu_write(ADDR_S_1, rh);
		data++;
		u16 ret = modbus_rtu_read(ADDR_A_0);
		cmd_LCD(0x01,0);
		cmd_LCD(0x80,0);
		fprintf(lcd_stream,"%d", ret);
		_delay_ms(1000);
#endif
#if COOLER
		if(ret)
			PORTD = SET(DC_PIN);
		CLR_BIT(PORTD, DC_PIN);
		//valor_pwm = 180;
		//set_dutty(valor_pwm);
		//valor_pwm+=10;
		//_delay_ms(1000);
#endif
	}
}
