/**
*	@brief		C++ library Arduino framework for Bosch sensors: BMP280, BME280, BME680, via i2c.
*	@author		Igor Mkprog, mkprogigor@gmail.com
*	@version	V1.1	@date	10.10.2025
*	@example	https://github.com/mkprogigor/mkigor_BMxx80/blob/main/examples/test_bme680.ino
*
*	@remarks	Glossary, abbreviations used in the module. Name has small or capital letters ("camelCase"),
*	and consist only 2 or 1 symbol '_', that divede it in => prefix + name + suffix.
*	prefix: 
*		gv_*	- Global Variable;
*		lv_*	- Local Variable (live inside statement);
*		cl_*	- CLass;
*		cd_*	- Class Definition;
*		cgv_*	- Class public (Global) member (Variable);
*		clv_*	- Class private (Local) member (Variable);
*		cgf_*	- Class public (Global) metod (Function), not need, no usefull, becouse we see parenthesis => ();
*		clf_*	- Class private (Local) metod (Function);
*		lp_*	- in function, local parameter.
*	suffix:
*		*_stru	- [like *_t] as usual, point to the type.
	example:	- prefix_nameOfFuncOrVar_suffix, gv_tphg_stru => global var (tphg) structure.
*/

#include <Arduino.h>
#include <Wire.h>

#ifndef mkigor_BMxx80_h
#define mkigor_BMxx80_h

#define cd_NOR_MODE		0x03
#define cd_FOR_MODE		0x01

#define cd_SB_500US		0x00
#define cd_SB_10MS		0x06
#define cd_SB_20MS		0x07
#define cd_SB_6250US	0x01
#define cd_SB_125MS		0x02
#define cd_SB_250MS		0x03
#define cd_SB_500MS		0x04
#define cd_SB_1000MS	0x05

#define cd_OS_OFF		0x00
#define cd_OS_x1		0x01
#define cd_OS_x2		0x02
#define cd_OS_x4		0x03
#define cd_OS_x8		0x04
#define cd_OS_x16		0x05

#define cd_FIL_OFF		0x00
#define cd_FIL_x2		0x01
#define cd_FIL_x4		0x02
#define cd_FIL_x8		0x03
#define cd_FIL_x16		0x04
#define cd_FIL_x32		0x05
#define cd_FIL_x64		0x06
#define cd_FIL_x128		0x07

#define cd_BMP280		0x58
#define cd_BME280		0x60
#define cd_BME680		0x61

struct tp_stru {
	float temp1;
	float pres1;
};
struct tph_stru {
	float temp1;
	float pres1;
	float humi1;
};
struct tphg_stru {
	float temp1;
	float pres1;
	float humi1;
	float gasr1;
};

//================================================
//		class cl_BMP280
//================================================
class cl_BMP280 {
private:
	uint8_t clv_i2cAddr;
	uint8_t clv_codeChip;
	struct {		/// clv_cd = structure of calibration data (coefficients)
		uint16_t	T1;
		int16_t		T2;
		int16_t		T3;
		uint16_t	P1;
		int16_t		P2;
		int16_t		P3;
		int16_t		P4;
		int16_t		P5;
		int16_t		P6;
		int16_t		P7;
		int16_t		P8;
		int16_t		P9;
	} clv_cd;
	void clf_readCalibData(void);	/// read calibration coeff, datas

public:
	cl_BMP280() {				///	default class constructor
		clv_i2cAddr = 0x77;		///	default BMP280 i2c address
		clv_codeChip = 0;		///	default code chip 0 => not found.
	}
	uint8_t readReg(uint8_t address);	/// read 1 byte from bme280 register by i2c
	bool				writeReg(uint8_t address, uint8_t data);	/// write 1 byte to bme280 register
	bool				reset(void);	/// bme280 software reset 
	uint8_t check(uint8_t lv_i2caddr);	/// function with parameter default value
	///	check sensor with i2c address or DEFAULT i2c address, return code chip
	void do1Meas(void);					/// DO 1 MEASurement and go to sleep (only for FORCED_MODE)
	bool isMeas(void);					/// returns TRUE while the bme280 IS MEASuring

	void begin();						/// init BMP280 with default parameters FORCED mode and max measuring 
	void begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p); // overloaded function init
	tp_stru readTP(void);				/// read, calculate and return structure T, P
};

//================================================
//						class cl_BME280, inherits cl_BMP280
//================================================
class cl_BME280 : public cl_BMP280 {
private:
	uint8_t clv_i2cAddr;
	uint8_t clv_codeChip;
	struct {			/// clv_cd = structure of calibration data (coefficients)
		uint16_t	T1;
		int16_t		T2;
		int16_t		T3;
		uint16_t	P1;
		int16_t		P2;
		int16_t		P3;
		int16_t		P4;
		int16_t		P5;
		int16_t		P6;
		int16_t		P7;
		int16_t		P8;
		int16_t		P9;
		uint8_t		H1;
		int16_t		H2;
		uint8_t		H3;
		int16_t		H4;
		int16_t		H5;
		int8_t		H6;
	} clv_cd;
	void clf_readCalibData(void);	/// read calibration coeff(data)

public:
	cl_BME280() {					/// default class constructor
		clv_i2cAddr = 0x76;			/// default BME280 i2c address
		clv_codeChip = 0;			/// default code chip 0 => not found.
	}

	void begin();	/// init BMx280 with default parameters FORCED mode and max measuring 
	void begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h); // overloaded function init
	tph_stru readTPH(void);			/// read, calculate and return structure T, P, H
};

//================================================
//	class cl_BME680, inherits cl_BMP280
//================================================
class cl_BME680 : public cl_BMP280 {
private:
	uint8_t clv_i2cAddr;
	uint8_t clv_codeChip;
	struct {	/// clv_cd = structure of calibration data (coefficients)
		uint16_t 	T1;
		int16_t		T2;
		int8_t		T3;

		uint16_t	P1;
		int16_t		P2;
		int8_t		P3;
		int16_t		P4;
		int16_t		P5;
		int8_t		P6;
		int8_t		P7;
		int16_t		P8;
		int16_t		P9;
		uint8_t		P10;

		uint16_t	H1;
		uint16_t	H2;
		int8_t		H3;
		int8_t		H4;
		int8_t		H5;
		uint8_t		H6;
		int8_t		H7;

		int8_t		G1;
		int16_t		G2;
		int8_t		G3;
	} clv_cd;
	void clf_readCalibData(void);	/// read calibration coeff(data)

public:
	cl_BME680() {				/// default class constructor
		clv_i2cAddr = 0x77;		/// default BME280 i2c address
		clv_codeChip = 0;		/// default code chip 0 => not found.
	}
	void initGasPointX(uint8_t point = 0, uint16_t tagTemp = 350, uint16_t duration = 100, int16_t ambTemp = 20);
	void do1Meas(void);			/// mode FORCED_MODE DO 1 Measuring}
	bool isMeas(void);			/// returns TRUE while bme680 is Measuring
	void begin();				/// init BMx280 with default parameters FORCED mode and max measuring 
	void begin(uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h); // overloaded function
	tphg_stru readTPHG(void);	/// read, calculate and return structure T, P, H, G
};

#endif

//=================================================================================