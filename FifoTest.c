#include <Wire.h>
#include <EEPROM.h>

void setup()
{
	Serial.begin(57600);
	Wire.begin();
	TWBR = 12;
	
	
	Wire.beginTransmission(gyro_address);
	Wire.write(0x6B);
	Wire.write(0x00);			//Set PWR_MGMT_1 = 00000000 to activate the gyro
	Wire.endTransmission();
	Wire.beginTransmission(gyro_address);
	Wire.write(0x1B);
	Wire.write(0x08);			//Set GYRO_CONFIG = 00001000 (500dps full scale)
	Wire.endTransmission();
	Wire.beginTransmission(gyro_address);
	Wire.write(0x1C);
	Wire.write(0x10);			//Set ACCEL_CONFIG = 00010000 (+/- 8g full scale range)
	Wire.endTransmission();
	Wire.beginTransmission(gyro_address);
	Wire.write(0x1A);
	Wire.write(0x03);			//Set CONFIG = 00000011 (Set Digital Low Pass Filter to ~43Hz)
	Wire.endTransmission();
	
	Wire.beginTransmission(gyro_address);
	Wire.write(0x23);
	Wire.write(0b01111000);  //Enable XG,YG,ZG,ACCEL
	Wire.endTransmission();
	Wire.beginTransmission(gyro_address);
	Wire.write(0x6A);
	Wire.write(0b01000000); //Enable Fifo
	Wire.endTransmission();
}
float acc_axis[4];
float gyro_axis[4];
uint8_t printCount = 0;

void fifoRead()
{
	do {
		Wire.beginTransmission(gyro_address);
		Wire.write(0x72);
		Wire.endTransmission();
		Wire.requestFrom(gyro_address, 2);
		while(Wire.available() < 2);
		
		uint16_t count = Wire.read()<<8|Wire.read();
	} while(count < 4 * 12);

	Wire.beginTransmission(gyro_address);
	Wire.write(0x74);
	Wire.endTransmission();
	Wire.requestFrom(gyro_address, 4 * 12);

	for(uint8_t i = 0; i < 4; ++i) {
		while(Wire.available() < 12);
		
		int16_t data = Wire.read()<<8|Wire.read();
		acc_axis[1] += data;
		int16_t data = Wire.read()<<8|Wire.read();
		acc_axis[2] += data;
		int16_t data = Wire.read()<<8|Wire.read();
		acc_axis[3] += data;
		int16_t data = Wire.read()<<8|Wire.read();
		gyro_axis[1] += data / 65.5;
		int16_t data = Wire.read()<<8|Wire.read();
		gyro_axis[2] += data / 65.5;
		int16_t data = Wire.read()<<8|Wire.read();
		gyro_axis[3] += data / 65.5;
	}
	
	if(++printCount >= 100 ) {
		printCount = 0;
		Serial.print("Gyro: ");
		Serial.print(gyro_axis[1]);
		Serial.print(",\t");
		Serial.print(gyro_axis[2]);
		Serial.print(",\t");
		Serial.print(gyro_axis[3]);
		Serial.println();
		
		Serial.print("Acc: ");
		Serial.print(acc_axis[1]);
		Serial.print(",\t");
		Serial.print(acc_axis[2]);
		Serial.print(",\t");
		Serial.print(acc_axis[3]);
		Serial.println();
	}
	
	acc_axis[1] = acc_axis[2] = acc_axis[3] = 0;
	gyro_axis[1] = gyro_axis[2] = gyro_axis[3] = 0;
}