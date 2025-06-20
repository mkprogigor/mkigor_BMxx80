/*
	Example of using mkigor_BMx280
*/

#include <Arduino.h>
#include "mkigor_BMx280.h"

BMx280 bme;
struct_tph result;

void setup() {
	Serial.begin(115200);

	uint8_t k = bme.check(0x76);
	if (k == 0) {
		Serial.println("BME280 not found. Check wires conection.");	
	}
	else {
		Serial.print("BME280 is present. Chip code = ");
		Serial.println(k, HEX);
	}
	bme.begin();
}

void main() {
	bme.do1meas();
	result = bme.read_TPH();

	Serial.print("Temperature, C = ");
	Serial.println(result.temp1);

	Serial.print("Pressure, Pa = ");
	Serial.println(result.pres1);

	Serial.print("Humidity, % = ");
	Serial.println(result.humi1);

	delay(3000);
}
