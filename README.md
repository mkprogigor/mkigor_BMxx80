# mkigor_BMx280
Lightweight and short library for Bosch sensor BME280 and BMP280 (maybe BME680) for Arduino.
It is not pretende to the most optimal code, but is example of learning C++ OOP.
Classes, Encapsulating metods (functions), Inheritance classes.
It has two classes: cl_BMP280 and cl_BME280. Class BME280 inherit BMP280.

By default, lib focused on one time measuring (forced mode, but normal available too) 
with max oversampling x16 of T,P,H, and filter x16. It's suitable for weather station.

Function => `uint8_t check(uint8_t _lv_i2caddr)`<BR>
It must be call 1st, becouse it checks present sensor!<BR>
Fn check connection with i2c sensor by I2C address `_lv_i2caddr` of sensor BME280 or BMP280 (0x76 or 0x77).<BR>
Fn return byte: 0 if not present sensor or CHIP CODE.<BR>
CHIP CODEs: 0x58=>BMP280, 0x60=>BME280, 0x61=>BME680. Note: i2c address 0x77 possible for BMP280 or BME280 or BME680 or MS5607,MS5611,MS5637, - check it.<BR>

Function => `void begin()`<BR>
setup bmp280 or bme280 to FORCED MODE, with x16 oversampling for T,P,H and x16 filter.<BR>

Function => `void begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h)`<BR>
init bme280 with Your parameters:<BR>
mode = NOR_MODE or FOR_MODE; <BR>
t_sb = standby (sleep) time, from SB_500US to SD_1000MS;<BR>
filter  =  value of filterring - FIL_OFF, FIL_x2, FIL_x4, FIL_x8, FIL_x16;<BR>
osrs_t, osrs_p, osrs_h = oversampling value for T,P,H - OS_OFF, OS_x1, OS_x2, OS_x4, OS_x8, OS_x16.<BR>

Function => `void do1Meas(void)`<BR>
Makes 1 measurement and goes to sleep (FORCED MODE).
You should make delay for waitung result.
Functions don't use of delay, it should do delay manually or usuing CPU resources for checkking results (`isMeas()`).
You can use vTaskDelay(200) for benefits of using FreeRTOS.
Max time measuring takes about 200 mS, acording to mode, sleep time, filter and oversamlinhg value
But You can check it.<BR>

Function => `bool isMeas(void)`<BR>
returns TRUE while the bme280 IS MEASuring, otherwise FALSE.<BR>

```c++
struct tph_srtu {
  float temp1;
  float pres1;
  float humi1;
};
```
Function => `tph_stru readTPH(void)`<BR>
It return values of T,P,H in structure variable.
It DOES NOT make measurement, only read in one I2C request, decoding to value T,P,H and return it in structure variable.<BR>

I used oficial Bosch datasheet bmp280, bme280 and examples for making this lib:<BR>
https://github.com/GyverLibs/GyverBME280<BR>
https://github.com/farmerkeith/BMP280-library/<BR>
https://github.com/farmerkeith/I2CScanner/<BR>
