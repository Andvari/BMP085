/*
 * bmp085.h
 *
 *  Created on: Sep 22, 2012
 *      Author: nemo
 */

#ifndef BMP085_H_
#define BMP085_H_

#define	BMP085_NUM_REGS		11
#define BMP085_BASE_ADDRESS	0x77

#define	AC1_ADDR	0xAA
#define	AC2_ADDR	0xAC
#define	AC3_ADDR	0xAE
#define	AC4_ADDR	0xB0
#define	AC5_ADDR	0xB2
#define	AC6_ADDR	0xB4
#define	B1_ADDR		0xB6
#define	B2_ADDR		0xB8
#define	MB_ADDR		0xBA
#define	MC_ADDR		0xBC
#define	MD_ADDR		0xBE

#include <Arduino.h>
#include <inttypes.h>
#include <Wire.h>

class BMP085{
public:
	BMP085(void);

	void	init(void);
	void	begin(void);

	void	setMode(uint8_t);
	uint8_t getMode(void);

	int32_t	getTemperature(void);
	int32_t	getPressure(void);

private:
	void	getReg(int16_t *,  const uint8_t);


	TwoWire		Wire;

	int16_t		AC1;
	int16_t 	AC2;
	int16_t 	AC3;
	uint16_t	AC4;
	uint16_t	AC5;
	uint16_t	AC6;
	int16_t		B1_;
	int16_t		B2_;
	int16_t		MB;
	int16_t		MC;
	int16_t		MD;
	int32_t		B5_;

	uint8_t oss;
};

#endif /* BMP085_H_ */
