/*
 * bmp085.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nemo
 */


#include "bmp085.h"

BMP085 :: BMP085(void){
}

void BMP085 :: init(void){

	  Wire.begin();

	  getReg(&AC1, AC1_ADDR);
	  getReg(&AC2, AC2_ADDR);
	  getReg(&AC3, AC3_ADDR);
	  getReg((int16_t *)&AC4, AC4_ADDR);
	  getReg((int16_t *)&AC5, AC5_ADDR);
	  getReg((int16_t *)&AC6, AC6_ADDR);
	  getReg(&B1_, B1_ADDR);
	  getReg(&B2_, B2_ADDR);
	  getReg(&MB, MB_ADDR);
	  getReg(&MC, MC_ADDR);
	  getReg(&MD, MD_ADDR);

	  oss = 0;
}

void BMP085 :: begin(void){
}

void BMP085 :: setMode(uint8_t new_oss){
	oss = new_oss;
}

uint8_t BMP085 :: getMode(void){
	return oss;
}

int32_t BMP085 :: getTemperature(void){
	uint8_t MSB;
	uint8_t LSB;
	int32_t UT;
	int32_t X1;
	int32_t X2;


    Wire.beginTransmission(BMP085_BASE_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x2E);
    Wire.endTransmission(false);
    delay(5);

    Wire.beginTransmission(BMP085_BASE_ADDRESS);
    Wire.write(0xF6);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP085_BASE_ADDRESS, 2, false);
    MSB = Wire.read();
    LSB = Wire.read();
    Wire.endTransmission(true);

    UT = MSB << 8 ;
    UT += LSB;
    X1  = (UT - AC6)*AC5/32768l;
    X2  = MC * 2048l / (X1 + MD);
    B5_ = X1 + X2;

/*
    Serial.print("UT: ");  Serial.print(UT);   Serial.print("\n");
    Serial.print("X1: ");  Serial.print(X1);   Serial.print("\n");
    Serial.print("X2: ");  Serial.print(X2);   Serial.print("\n");
    Serial.print("B5: ");  Serial.print(B5_);  Serial.print("\n");
    Serial.print("T: ");   Serial.print(T);    Serial.print("\n");
*/

    return (B5_ + 8l)/16l;
}

int32_t BMP085 :: getPressure(void){
	int8_t		MSB;
	int8_t		LSB;
	int8_t		XLSB;
	int32_t		UP;
	int32_t		B3_;
	uint32_t	B4_;
	int32_t		B6_;
	int32_t		B7_;
	int32_t		X1;
	int32_t		X2;
	int32_t		X3;
	int32_t		p;


    Wire.beginTransmission(0x77);
    Wire.write(0xF4);
    Wire.write(0x34 + (oss << 6));
    Wire.endTransmission(false);
    delay(8);

    Wire.beginTransmission(0x77);
    Wire.write(0xF6);
    Wire.endTransmission(false);
    Wire.requestFrom(0x77, 1, false);
    MSB = Wire.read();
    Wire.endTransmission(true);

    Wire.beginTransmission(0x77);
    Wire.write(0xF7);
    Wire.endTransmission(false);
    Wire.requestFrom(0x77, 1, false);
    LSB = Wire.read();
    Wire.endTransmission(true);

    Wire.beginTransmission(0x77);
    Wire.write(0xF8);
    Wire.endTransmission(false);
    Wire.requestFrom(0x77, 1, false);
    XLSB = Wire.read();
    Wire.endTransmission(true);


    UP  = (long)MSB << 16;
    UP  |= (long)LSB << 8;
    UP  |= (long)XLSB;
    UP  = UP >> (8-oss);
    B6_ = B5_ - 4000l;
    X1  = (B2_ * (B6_ * B6_ / 4096l)) / 2048l;
    X2  = AC2 * B6_ / 2048l;
    X3  = X1 + X2;
    B3_ = AC1 * 4l + X3;
    B3_ = B3_ << oss;
    B3_ += 2l;
    B3_ /= 4l;

    X1  = AC3 * B6_ / 8192;
    X2  = (B1_ * (B6_ * B6_ / 4096)) / 65536;
    X3  = ((X1 + X2) + 2) / 4;
    B4_ = AC4 * (unsigned long)(X3 + 32768) / 32768;
    B7_ = ((unsigned long)UP - B3_) * (50000 >> oss);
    if(B7_ < (int32_t)0x80000000){ p = (B7_ * 2)/ B4_;}
    else { p = (B7_ / B4_) * 2;}
    X1 = (p / 256) * (p / 256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;

  /*
    Serial.print("MSB:  ");  Serial.print(MSB);   Serial.print("\n");
    Serial.print("LSB:  ");  Serial.print(LSB);   Serial.print("\n");
    Serial.print("XLSB: ");  Serial.print(XLSB);   Serial.print("\n");

    Serial.print("UP: ");  Serial.print(UP);   Serial.print("\n");
    Serial.print("B5: ");  Serial.print(B5_);  Serial.print("\n");
    Serial.print("B6: ");  Serial.print(B6_);  Serial.print("\n");
    Serial.print("B3: ");  Serial.print(B3_);  Serial.print("\n");
    Serial.print("X1: ");  Serial.print(X1);   Serial.print("\n");
    Serial.print("X2: ");  Serial.print(X2);   Serial.print("\n");
    Serial.print("X3: ");  Serial.print(X3);   Serial.print("\n");
    Serial.print("B4: ");  Serial.print(B4_);   Serial.print("\n");
    Serial.print("B7: ");  Serial.print(B7_);  Serial.print("\n");
    Serial.print("p: ");   Serial.print(p);    Serial.print("\n");
*/

	return p + (X1 + X2 + 3791) / 16;
}

void BMP085 :: getReg(int16_t *dst, uint8_t addr){
	uint8_t t1;
	uint8_t t2;

    Wire.beginTransmission(BMP085_BASE_ADDRESS);
    Wire.write(addr);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP085_BASE_ADDRESS, 1, false);
    t1 = Wire.read();
    Wire.endTransmission(true);

    Wire.beginTransmission(BMP085_BASE_ADDRESS);
    Wire.write(addr+1);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP085_BASE_ADDRESS, 1, false);
    t2 = Wire.read();
    Wire.endTransmission(true);
    *dst = (t1 << 8) + t2;
}
