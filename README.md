# bme280
Lightweight and short library for Bosch sensor BME280 (in development).

It operate and return integer values of t,p,h at once.
Further you can convert to float in calling procedure.
By default, lib focused on one time measuring (forced mode, normal available too) 
with max oversampling x16 of t,p,h, and filter x16. It's suitable for weather station.

Function bme280.f_read_TPH() is reading value t,p,h for one I2C request and return structure:
```c++
struct struct_tph {
  int32_t  temp1;  //  real t = t * 100
  unt32_t  pres1;  //  real p = p * 100
  unt32_t  humi1;  //  real h = h * 1000
};
```
real values can get: T = ((float)(struct_tph.temp1)) / 100.

It is not use functions of delay, it should do it manually for benefits of using FreeRTOS.
Or usuing CPU resources for checkking results. Max measuring time takes about 200 mS, but You can check it.
