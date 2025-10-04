# mkigor_BMx280
Lightweight and short library for Bosch sensor BME280 and BMP280 (maybe BME680) for Arduino.
It is not pretend to the most optimal code, but it is example of learning C++ OOP for programing MCU.
Classes, Encapsulating metods (functions), Inheritance classes.
It has two classes: cl_BMP280 and cl_BME280. Class BME280 inherit BMP280.

By default, lib focused on one time measuring (forced mode, but normal available too) 
with max oversampling x16 of T,P,H, and filter x16. It's suitable for weather station.

Function => `uint8_t check(uint8_t lv_i2caddr)`<BR>
It must be call 1st, because it checks presence of sensor!<BR>
Fn check connection with sensor BME280 or BMP280 by I2C address `lv_i2caddr` (0x76 or 0x77).<BR>
Fn return byte: 0 = if sensor does not present or CHIP CODE in otherwise.<BR>
Possible chip codes are: 0x58=>BMP280, 0x60=>BME280, 0x61=>BME680.<BR>
Note: i2c address 0x77 possible for BMP280 or BME280 or BME680 or MS5607,MS5611,MS5637, - check it.<BR>

Function => `void begin()`<BR>
Default setup for bmp280 or bme280: FORCED MODE, with x16 oversampling and x16 filter.<BR>

Function => `void begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h)`<BR>
Default setup for bmp280 or bme280 with Your parameters:<BR>
mode = NOR_MODE or FOR_MODE; <BR>
t_sb = standby (sleep) time, from SB_500US to SD_1000MS;<BR>
filter  =  value of filterring - FIL_OFF, FIL_x2, FIL_x4, FIL_x8, FIL_x16;<BR>
osrs_t, osrs_p, osrs_h = oversampling value for T,P,H - OS_OFF, OS_x1, OS_x2, OS_x4, OS_x8, OS_x16.<BR>

Function => `void do1Meas(void)`<BR>
Makes 1 measurement and goes to sleep (FORCED MODE).<BR>
The function don't use delay or wait for result, only send command to sensor - start measuring.<BR>
You should do delay and check moment (`isMeas()`) when measuring data will be finish.<BR>
You can use vTaskDelay(200) for benefits of using FreeRTOS.<BR>
Max time measuring takes about 200 mS, acording to mode, sleep time, filter and oversamlinhg value.<BR>
But You can check it.<BR>

Function => `bool isMeas(void)`<BR>
returns TRUE while the bmp280 or bme280 IS MEASuring, otherwise FALSE.<BR>

```c++
struct tph_srtu {
  float temp1;
  float pres1;
  float humi1;
};
```
Function => `tph_stru readTPH(void)`<BR>
This metod (function) DOES NOT make measurement!<BR>
The function only reads data in one I2C request,<BR>
decoding to value T,P,H and return it in structure variable.<BR>

I used oficial Bosch datasheet bmp280, bme280, bme680. But datasheets have errors, I finded working code in next libs, becouse THE CODE IS THE DOCUMENTATION :-) I thanks authors for help in coding:<BR>
https://github.com/GyverLibs/GyverBME280<BR>
https://github.com/farmerkeith/BMP280-library/<BR>
https://github.com/boschsensortec/BME280_SensorAPI<BR>
Also thanks for help in bme680:<br>
https://github.com/Zanduino/BME680<br>
https://github.com/kriswiner/BME680<br>
https://github.com/boschsensortec/BME68x_SensorAPI<BR>
https://github.com/adafruit/Adafruit_BME680<BR>

<BR>
Example of use this lib in my project of "Easy DIY weather station"<BR>
https://github.com/mkprogigor/mkigor_esp32c3_ws<BR>
