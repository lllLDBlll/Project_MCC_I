/*
 * modbus.h
 *
 * Created: 14/12/2018 15:04:53
 *  Author: Leonardo D. Batista
 */ 

#ifndef MODBUS_RTU_H_
#define MODBUS_RTU_H_

#define ESP_ADDR	0x15
#define W_CMD		0x01
#define R_CMD		0x02
#define BAUD		B9600
#define LEN			8

//Read-Only Registers
#define ADDR_A_0	0x01
#define ADDR_A_1	0x02
#define ADDR_A_2	0x03
#define ADDR_A_3	0x04

//Write Registers
#define ADDR_S_0	0x05
#define ADDR_S_1	0x06
#define ADDR_S_2	0x07
#define ADDR_S_3	0x08

/*
+-----------+-----------+-----------+-----------+-----------+
|	addr	|	cmd		|	reg		|	data	|	crc		|
+-----------+-----------+-----------+-----------+-----------+
|	1byte	|	1byte	|	2byte	|	2byte	|	2byte	|	
+-----------+-----------+-----------+-----------+-----------+
|			|			|	big en	|	big en  |	big en	|
+-----------+-----------+-----------+-----------+-----------+
*/
typedef struct frame frame_t;

void modbus_rtu_init();
//frame_t *packeted(uint16_t);
uint8_t *modbus_rtu_read();
//void modbus_rtu_write(frame_t *, int);
void modbus_rtu_write(uint16_t , uint16_t);
void modbus_rtu_print(uint16_t );
void nibble_data(uint8_t *);
uint16_t CRC16_2(uint8_t * , int );

#endif /* MODBUS_RTU_H_ */