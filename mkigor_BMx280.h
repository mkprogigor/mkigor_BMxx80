/************************************************************************************
Library for test and use Bosch i2c sensor bme280, bmp280
by Igor Mkprog, mkprogigor@gmail.com

use examples:
https://github.com/AlexGyver/GyverLibs/blob/master/GyverBME280/
https://github.com/farmerkeith/BMP280-library/
https://github.com/farmerkeith/I2CScanner/

V1.2 from 19.06.2025
************************************************************************************/

#ifndef mkigor_bme280_h
#define mkigor_bme280_h

#include <Arduino.h>
#include <Wire.h>

#define NOR_MODE  0x03
#define FOR_MODE  0x02

#define SB_500US  0x00
#define SB_10MS   0x06
#define SB_20MS   0x07
#define SB_6250US 0x01
#define SB_125MS  0x02
#define SB_250MS  0x03
#define SB_500MS  0x04
#define SB_1000MS 0x05

#define OS_OFF  0x00
#define OS_x1   0x01
#define OS_x2   0x02
#define OS_x4   0x03
#define OS_x8   0x04
#define OS_x16  0x05

#define FIL_OFF 0x00
#define FIL_x2  0x01
#define FIL_x4  0x02
#define FIL_x8  0x03
#define FIL_x16 0x04

#define BMP280  0x58
#define BME280  0x60
#define BME680  0x61

struct struct_tph {
    float temp1;
    float pres1;
    float humi1;
};

/*************************************************/
class BMx280 {
    public:
        BMx280();                           // create an object of class bme280
        uint8_t check(uint8_t _lv_i2caddr); // check sensor with i2c address, return code chip
        bool begin();                       // default parameters
        bool begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h); // init bme280
        void do1meas(void);             // do 1 measurement and go to sleep (only for FORCED_MODE)
        bool is_meas(void);             // returns TRUE while the bme280 IS measuring					
        struct_tph read_TPH(void);      // read, calculate and return int structure T*100, P*100, H*1000
        uint8_t gv_i2c_address = 0x76;
        uint8_t gv_code_chip;
    
    private:
        uint8_t _f_read_reg(uint8_t address);   // read 1 byte from bme280 register by i2c
        bool _f_write_reg(uint8_t address , uint8_t data);  // write 1 byte to bme280 register
        bool _f_reset(void);                    // bme280 software reset 
        void _f_read_calibr_coeff(void);        // read calibration coeff
        struct {                                // structure of calibration coefficients
            uint16_t  dig_T1;
            int16_t   dig_T2;
            int16_t   dig_T3;
            uint16_t dig_P1;
            int16_t  dig_P2;
            int16_t  dig_P3;
            int16_t  dig_P4;
            int16_t  dig_P5;
            int16_t  dig_P6;
            int16_t  dig_P7;
            int16_t  dig_P8;
            int16_t  dig_P9;
            uint8_t dig_H1;
            int16_t dig_H2;
            uint8_t dig_H3;
            int16_t dig_H4;
            int16_t dig_H5;
            int8_t  dig_H6;
        } _cd ;
};

#endif
