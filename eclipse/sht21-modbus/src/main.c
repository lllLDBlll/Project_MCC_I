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
#include "../lib/avr_timer.h"
#include "../lib/avr_usart.h"
#include "../lib/avr_gpio.h"
#include "../lib/lcd.h"
/*My includes*/
#include "../lib/modbus_rtu.h"
#include "../lib/sht21.h"

#define F_CPU 16000000UL //Define CPU Clock
#define HIGH 1
#define LOW 0

#define MQTT 0	//test MQTT by ESP01
#define PWM 0
#define SHT21 1

uint8_t I2C_buf[10]={0};
float tc, rh;

#if PWM
void timer2_pwm_hardware_init(){
	//fOCnxPWM = (fclk_I/O)/(N*256)
	/* PD5: pino OC0B como saída */
	GPIO_D->DDR |= SET(PD3);

	/* Table 15-6.  Compare Output Mode, Fast PWM Mode */
	/* COM0B1   COM0B0  Desc:
	    0       0       Normal port operation, OC0B disconnected.
	    0       1       Reserved
	    1       0       Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode)
	    1       1       Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode).*/

	/* WGM02, WGM01 WGM00 setados: modo PWM rápido com TOP em OCRA */
	TIMER_2->TCCRA = SET(WGM01) | SET(WGM00) | SET(COM0B1);
	TIMER_2->TCCRB = SET(WGM02) | SET(CS00);

	/* OCRA define frequência do PWM */
	TIMER_2->OCRA = 200;

	/* OCRB define razão cíclica:  OCRB / OCRA */
	TIMER_2->OCRB = 150;
}

inline void set_dutty(uint8_t dutty){
	if (dutty <= TIMER_2->OCRA)
		TIMER_2->OCRB = dutty;
}
#endif

int main(void){
#if MQTT
	uint8_t pkg[8] = {0x15, 0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_pkg[16],i;
	uint16_t data = 0;
	uint16_t crc;

	//FILE *lcd_stream;

	/* Inicializa hardware da USART */
	//USART_Init(B9600);

	//lcd_stream = inic_stream();

	//inic_LCD_4bits();
	modbus_rtu_init();
	//fprintf(lcd_stream,"%s", "START!");
	_delay_ms(1000);
#endif

#if SHT21
	//Setup
	//TWI_Master_Initialise();
	uint8_t x = 0;

	/* Obtem o stream de depuração */
	FILE *debug = get_usart_stream();

	/* Inicializa hardware da USART */
	USART_Init(B9600);

	/* Mensagem incial: terminal do Proteus
	 * utiliza final de linha com '\r' */

	sht21_init();

#endif
#if PWM
	uint8_t valor_pwm = 1;

	/* Configura timer em modo PWM */
	timer2_pwm_hardware_init();
	valor_pwm = 199;
	set_dutty(valor_pwm);
	//sei();
	/* ExeMplo: aumenta em 10 a razão cíclica do PWM a cada 100ms */
#endif
	while(1){
#if SHT21
		/*
		I2C_buf[0] = SHT21_WRITE;
		I2C_buf[1] = T_NO_HOLD;
		TWI_Start_Transceiver_With_Data(I2C_buf, 2);

		_delay_ms(100);

		I2C_buf[0] = SHT21_READ;
		TWI_Start_Transceiver_With_Data(I2C_buf, 6);//4
		TWI_Get_Data_From_Transceiver(I2C_buf, 6);//4

		for(u8 i=0;i<6;i++){
			fprintf(debug, "0x%x\n\r", I2C_buf[i]);
		}
		 */

		uint16_t temp_value = sht21_read(T_NO_HOLD); //I2C_buf[1]<<8 | I2C_buf[2]>>2;
		tc = -46.85 + (175.72 / 65536.0) * temp_value;
		char buffer1[4];
		dtostrf(tc,5,2,buffer1);
		fprintf(debug, "TC = %s *C \n\r", buffer1);
		/*
		I2C_buf[0] = SHT21_WRITE;
		I2C_buf[1] = RH_NO_HOLD;
		TWI_Start_Transceiver_With_Data(I2C_buf, 2);

		_delay_ms(100);

		I2C_buf[0] = SHT21_READ;
		TWI_Start_Transceiver_With_Data(I2C_buf, 4);
		TWI_Get_Data_From_Transceiver(I2C_buf, 4);
		 */
		char buffer2[4];
		uint16_t hum_value = sht21_read(RH_NO_HOLD);//I2C_buf[1]<<8 | I2C_buf[2]>>2;
		rh = -6 + 125.0 / 65536.0 * hum_value;
		dtostrf(rh,5,2,buffer2);
		fprintf(debug, "RH = %s %%  \n\r", buffer2);

		fprintf(debug, "%s\n\r","----------");
		_delay_ms(1000);
#endif

#if MQTT
		modbus_rtu_write(ADDR_S_0, data);
		data++;
		u16 ret = modbus_rtu_read(ADDR_A_0);
		cmd_LCD(0x01,0);
		cmd_LCD(0x80,0);
		fprintf(lcd_stream,"%d", ret);
		_delay_ms(1000);
#endif
#if PWM
		//valor_pwm = 180;
		//set_dutty(valor_pwm);
		//valor_pwm+=10;
		//_delay_ms(1000);
#endif
	}
}
