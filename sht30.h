/*
 * sht30.h
 *
 *  Created on: 21 θών. 2018 γ.
 *      Author: iho
 */

#ifndef SHT30_H_
#define SHT30_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "Wire.h"

class SHT3X{
public:
	SHT3X(uint8_t address=0x45);
	byte get(void);
	float cTemp=0;
	float fTemp=0;
	float humidity=0;

private:
	uint8_t _address;

};

#endif /* SHT30_H_ */
