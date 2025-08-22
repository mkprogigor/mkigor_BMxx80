/************************************************************************************
Library for test and use Bosch i2c sensor bme280, bmp280
by Igor Mkprog, mkprogigor@gmail.com

use examples:
https://github.com/AlexGyver/GyverLibs/blob/master/GyverBME280/
https://github.com/farmerkeith/BMP280-library/
https://github.com/farmerkeith/I2CScanner/

V1.3 from 22.08.2025

Glossary, abbreviations used in the module, prefix and suffix:
gv_*    -   global variable;
lv_*    -   local variable (live inside statement);
cl_*    -   class;
cd_*    -   class definishion;
cgv_*   -   class public (global) member (variable);
clv_*   -   class private (local) member (variable);
cf_*    -   class public function (metod), not need, no usefull, becouse we see parenthesis => ();
clf_*   -   class private (local) metod (function);

*_stru  -   suffix, as usual, point the type.
************************************************************************************/

#ifndef mkigor_BMx280_h
#define mkigor_BMx280_h

#include <Arduino.h>
#include <Wire.h>

#define cd_NOR_MODE  0x03
#define cd_FOR_MODE  0x02

#define cd_SB_500US  0x00
#define cd_SB_10MS   0x06
#define cd_SB_20MS   0x07
#define cd_SB_6250US 0x01
#define cd_SB_125MS  0x02
#define cd_SB_250MS  0x03
#define cd_SB_500MS  0x04
#define cd_SB_1000MS 0x05

#define cd_OS_OFF  0x00
#define cd_OS_x1   0x01
#define cd_OS_x2   0x02
#define cd_OS_x4   0x03
#define cd_OS_x8   0x04
#define cd_OS_x16  0x05

#define cd_FIL_OFF 0x00
#define cd_FIL_x2  0x01
#define cd_FIL_x4  0x02
#define cd_FIL_x8  0x03
#define cd_FIL_x16 0x04

#define cd_BMP280  0x58
#define cd_BME280  0x60
#define cd_BME680  0x61

struct gv_tp_stru {
    float temp1;
    float pres1;
};
struct gv_tph_stru {
    float temp1;
    float pres1;
    float humi1;
};

/*************************************************/
//      class cl_BMP280
/*************************************************/
class cl_BMP280 {
private:
    uint8_t clv_i2cAddr;
    uint8_t clv_codeChip;
    struct {                                // clv_cd = structure of calibration data (coefficients)
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
    } clv_cd;
    void    clf_readCalibCoef(void);                        // read calibration coeff

public:
    cl_BMP280() {           // default class constructor
        clv_i2cAddr  = 0x77;    //  default BMP280 i2c address
        clv_codeChip = 0;       //  default code chip 0 => not found.
    }
    uint8_t readReg(uint8_t address);                   // read 1 byte from bme280 register by i2c
    bool    writeReg(uint8_t address, uint8_t data);    // write 1 byte to bme280 register
    bool    reset(void);                                // bme280 software reset 
    uint8_t check(uint8_t _lv_i2caddr = clv_i2cAddr);   // function with parameter default value
                        // check sensor with i2c address or DEFAULT i2c address, return code chip
    void do1meas(void);         // do 1 measurement and go to sleep (only for FORCED_MODE)
    bool is_meas(void);         // returns TRUE while the bme280 IS measuring					

    bool begin();               // init BMx280 with default parameters FORCED mode and max measuring 
    bool begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h); // overloaded function init
    gv_tp_stru read_tp(void);   // read, calculate and return int structure T*100, P*100

};

/*************************************************/
//      class cl_BME280, inherits cl_BMP280
/*************************************************/
class cl_BME280: public cl_BMP280 {
private:
    uint8_t clv_i2cAddr;
    uint8_t clv_codeChip;
    struct {                                // clv_cd = structure of calibration data (coefficients)
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
    } clv_cd;
    void    clf_readCalibCoef(void);                        // read calibration coeff

public:
    cl_BME280() {           // default class constructor
        clv_i2cAddr  = 0x76;    //  default BME280 i2c address
        clv_codeChip = 0;       //  default code chip 0 => not found.
    }

    bool begin();               // init BMx280 with default parameters FORCED mode and max measuring 
    bool begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h); // overloaded function init
    gv_tph_stru read_tph(void); // read, calculate and return int structure T*100, P*100, H*1000

};

#endif
