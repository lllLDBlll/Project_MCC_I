/*
 * sht21.h
 *
 * Created: 28/06/2019 10:02:25
 *  Author: Leonardo D. Batista
 */ 

#ifndef SHT21_H_
#define SHT21_H_
/****************************************************************************
  TWI Status/Control register definitions
****************************************************************************/
#define WD_ENABLE // Watch Dog Enable
#define HOLD_MASTER // Enables Hold Master Mode, if disables No Hold Master

/****************************************************************************
  Bit and byte definitions
****************************************************************************/
#define SHT21_WRITE 0x80 // SHT21 I2C Address 1000 0000 - 0x80 + Write
#define SHT21_READ 0x81 // SHT21 I2C Address 1000 0001 - 0x81 + Read

/*             Basic commands set table
+-------------------------+----------------+-----------+
|		 Command		  |	    Comment    |   Code    |
+-------------------------+----------------+-----------+
|  Trigger T measurement  |   hold master  | 1110’0011 |
+-------------------------+----------------+-----------+
|  Trigger RH measurement |   hold master  | 1110’0101 |
+-------------------------+----------------+-----------+
|  Trigger T measurement  | no hold master | 1111’0011 |
+-------------------------+----------------+-----------+
|  Trigger RH measurement | no hold master | 1111’0101 |
+-------------------------+----------------+-----------+
|  Write user register    |				   | 1110’0110 |
+-------------------------+----------------+-----------+
|  Read user register     |				   | 1110’0111 |
+-------------------------+----------------+-----------+
|	  Soft reset          |				   | 1111’1110 |
+-------------------------+----------------+-----------+
*/

#define T_HOLD 0xE3
#define RH_HOLD 0xE5
#define T_NO_HOLD 0xF3
#define RH_NO_HOLD 0xF5
#define WRITE_REG 0xE6
#define READ_REG 0xE7
#define SOFT_RESET 0xFE

#define POLYNOMIAL 0x131
/****************************************************************************
  Function definitions
****************************************************************************/

void sht21_init();
void sht21_write(unsigned char *, unsigned char);
uint16_t sht21_read(uint8_t reg);

#endif /* SHT21_H_ */
