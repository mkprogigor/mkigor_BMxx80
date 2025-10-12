/**
*  This is a example to use lib BMxx80.h/BMxx80.cpp
*  for the BME680 sensor BOSCH (temperature, pressure, humidity, gas)
*  These lib use only I2C interface. 
 ***************************************************************************/
#include <BMxx80.h>

cl_BME680 bme;  ///  create class

void setup() {
  Serial.begin(115200);

  uint8_t k = bme.check();
  k = bme.check(0x76);
  Serial.print("Check a bme280 => ");    /// check bme280 and SW reset
    if (k == 0) Serial.print("not found, check cables.\n");
  else {
    Serial.print(k, HEX);  Serial.println(" found chip code.");
  }
  bme.begin();
///  bme.begin(cd_FIL_OFF, cd_OS_x16, cd_OS_x16, cd_OS_x16);
  bme6.initGasPointX(0, 350, 100, 20);
}

void loop() {
  bme.initGasPointX(0, 350, 100, (int16_t)gv_stru_tph.temp1);
  bme.do1Meas();
  unsigned long lv_measStart = millis();
  for (uint8_t i = 0; i < 3000; i++) {
    if (bme6.isMeas()) delay(1);
    else break;
  }
  Serial.print("BME680 measuring TPHG time = ");
  Serial.println( millis()-lv_measStart );
  tphg_stru gv_stru_tphg = bme6.readTPHG();
  
  Serial.print("Temperature = ");
  Serial.print(gv_stru_tphg.temp1);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(gv_stru_tphg.pres1);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(gv_stru_tphg.humi1);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(gv_stru_tphg.gasr1);
  Serial.println(" KOhms");

  Serial.println();
  delay(3000);
}

