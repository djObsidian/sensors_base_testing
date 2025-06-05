/*
 * TCS3400.h
 *
 *  Created on: Mar 20, 2025
 *      Author: Admin
 */

#ifndef SENSORS_INC_TCS3400_H_
#define SENSORS_INC_TCS3400_H_

#define TCS3400_I2C_ADDR_13 0x39 //TCS34001, TCS34003
#define TCS3400_I2C_ADDR_7 0x29 //TCS34007

//Register map
#define TCS3400_REG_ENABLE 		0x80	//Enables states and interrupts
#define TCS3400_REG_ATIME 		0x81	//RGBC integration time
#define TCS3400_REG_WTIME 		0x83	//Wait time
#define TCS3400_REG_AILTL 		0x84	//Clear interrupt low threshold low byte
#define TCS3400_REG_AILTH 		0x85	//Clear interrupt low threshold high byte
#define TCS3400_REG_AIHTL 		0x86	//Clear interrupt high threshold low byte
#define TCS3400_REG_AIHTH 		0x87	//Clear interrupt high threshold high byte
#define TCS3400_REG_PERS 		0x8C	//Interrupt persistence filter
#define TCS3400_REG_CONFIG 		0x8D	//Configuration
#define TCS3400_REG_CONTROL		0x8F	//Gain control register
#define TCS3400_REG_AUX			0x90	//Auxiliary control register
#define TCS3400_REG_REVID		0x91	//Revision ID
#define TCS3400_REG_ID			0x92	//Device ID
#define TCS3400_REG_STATUS		0x93	//Device status
#define TCS3400_REG_CDATAL		0x94	//Clear / IR channel low data register
#define TCS3400_REG_CDATAH		0x95	//Clear / IR channel high data register
#define TCS3400_REG_RDATAL		0x96	//Red ADC low data register
#define TCS3400_REG_RDATAH		0x97	//Red ADC high data register
#define TCS3400_REG_GDATAL		0x98	//Green ADC low data register
#define TCS3400_REG_GDATAH		0x99	//Green ADC high data register
#define TCS3400_REG_BDATAL		0x9A	//Blue ADC low data register
#define TCS3400_REG_BDATAH		0x9B	//Blue ADC high data register
#define TCS3400_REG_IR			0xC0	//Access IR Channel
#define TCS3400_REG_IFORCE		0xE4	//Force Interrupt
#define TCS3400_REG_CICLEAR		0xE6	//Clear channel interrupt clear
#define TCS3400_REG_AICLEAR		0xE7	//Clear all interrupts


#endif /* SENSORS_INC_TCS3400_H_ */
