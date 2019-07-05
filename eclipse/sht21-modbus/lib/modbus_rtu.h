/*
 * modbus_rtu.h
 *
 * Created: 05/07/2019 02:18:53
 *  Author: Leonardo D. Batista
 */ 

#ifndef MODBUS_RTU_H_
#define MODBUS_RTU_H_

#define ESP_ADDR	0x15
#define W_CMD		0x01
#define R_CMD		0x02
#define BAUD		B9600

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
typedef uint8_t u8;
typedef uint16_t u16;

//typedef struct frame frame_t;

void modbus_rtu_init();

u16 modbus_rtu_read(u16 );

void modbus_rtu_write(u16 , u16);
void modbus_rtu_print(u16 );

u16 CRC16_2(u8 * , int );

#endif /* MODBUS_RTU_H_ */
