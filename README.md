# bme280
Lightweight and short library for Bosch sensor BME280 (test).

It operate and return integer values of t,p,h at once.
Further you can convert to real in calling procedure.
Lib focused on one time measuring (forced mode, normal available too) 
with max oversampling x16 of t,p,h, and filter x16.
Max measuring takes about 200 mS. Reading t,p,h takes one I2C request.
It's suitable for weather station.

Function bme280.f_read_TPH() return structure:
struct struct_tph {
  int32_t  temp1;  //  t * 100
  unt32_t  pres1;  //  p * 100
  unt32_t  humi1;  //  h * 1000
};
real values can get: ((float)(struct_tph.temp1)) / 100.
