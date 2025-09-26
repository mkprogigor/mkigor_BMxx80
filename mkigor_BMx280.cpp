/************************************************************************************
Library for test and use Bosch i2c sensor bme280, bmp280
by Igor Mkprog, mkprogigor@gmail.com

use examples:
https://github.com/AlexGyver/GyverLibs/blob/master/GyverBME280/
https://github.com/farmerkeith/BMP280-library/
https://github.com/farmerkeith/I2CScanner/

V1.0 from 30.08.2025

Glossary, abbreviations used in the module, prefix and suffix:
gv_*    -   Global Variable;
lv_*    -   Local Variable (live inside statement);
cl_*    -   CLass;
cd_*    -   Class Definishion;
cgv_*   -   Class public (Global) member (Variable);
clv_*   -   Class private (Local) member (Variable);
cgf_*   -   Class public (Global) metod (Function), not need, no usefull, becouse we see parenthesis => ();
clf_*   -   Class private (Local) metod (Function);

*_stru  -   [or *_stru_t] suffix, as usual, point the type.

Metods (functions) dont use symbol '_', only small or capital lett
************************************************************************************/
#include <mkigor_BMx280.h>

// #define enDEBUG

//============================================
//	common public metod (function) for cl_BMP280 and cl_BME280 and cl_BME680
//============================================
uint8_t cl_BMP280::readReg(uint8_t address) {    // read register 1 byte from bmp280 or bme280 
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(address);
	if (Wire.endTransmission() != 0) return 0;
	if (Wire.requestFrom(clv_i2cAddr, 1) == 1) return Wire.read();
	else return 0;
}

bool cl_BMP280::writeReg(uint8_t address, uint8_t data) {   //  write 1 byte to bme280 register
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(address);
	Wire.write(data);
	if (Wire.endTransmission() == 0) return true;
	else return false;
}

uint8_t cl_BMP280::check(uint8_t lv_i2caddr) {
	// check is it conected bm(e,p)280, if YES ret chip_code, if NO ret 0
	clv_i2cAddr = lv_i2caddr;
	Wire.begin();
	uint8_t lv_error;
    Wire.beginTransmission(clv_i2cAddr);
    lv_error = Wire.endTransmission();
    if (lv_error == 0) {
        Wire.beginTransmission(clv_i2cAddr);
        Wire.write(0xD0);   // Select register address = 0xD0 of chip_id
        Wire.endTransmission();
        Wire.requestFrom(clv_i2cAddr, 1);
		if (Wire.available() == 1) {
			clv_codeChip = Wire.read();
			reset();
			return clv_codeChip;
		}
    }
    return 0;
	/*  fn return chip codes: 0x58=BMP280, 0x60=BME280, 0x61=BME680.
	i2c address 0x76, 0x77 possible for BMP280 or BME280 or BME680, note: CHECK IT ! */
}

bool cl_BMP280::reset(void) {     //  software reset of bmp280, bme280, bme280. Return TRUE if write op. is OK
	if (cl_BMP280::writeReg(0x0E, 0xB6)) return true;
    else return false;
}

void cl_BMP280::do1Meas(void) {    // mode FORCED_MODE DO 1 Measuring
	uint8_t lv_reg_0xF4 = cl_BMP280::readReg(0xF4);
	cl_BMP280::writeReg(0xF4, ((lv_reg_0xF4 & 0xFC) | 0x01));
}

bool cl_BMP280::isMeas(void) {	// returns TRUE while bme280 is Measuring								
	return (bool)((cl_BMP280::readReg(0xF3) & 0x08) >> 3);  	 // read status register & mask bit "measuring"
}



//============================================
//	specific  metods (funcs) for cl_BMP280
//    private metods (funcs)
//============================================
void cl_BMP280::clf_readCalibCoef(void) {
	uint8_t lv_nregs = 24;
	uint8_t lv_regs[lv_nregs];		// temporary array for reading registers

	Wire.beginTransmission(clv_i2cAddr);   // first part request
	Wire.write(0x88);
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {    // reading 26 regs
		for (uint8_t i = 0; i < 25; i++) lv_regs[i] = Wire.read();

		clv_cd.T1 = lv_regs[1] << 8 | lv_regs[0];   // form struct
		clv_cd.T2 = lv_regs[3] << 8 | lv_regs[2];
		clv_cd.T3 = lv_regs[5] << 8 | lv_regs[4];
		clv_cd.P1 = lv_regs[7] << 8 | lv_regs[6];
		clv_cd.P2 = lv_regs[9] << 8 | lv_regs[8];
		clv_cd.P3 = lv_regs[11] << 8 | lv_regs[10];
		clv_cd.P4 = lv_regs[13] << 8 | lv_regs[12];
		clv_cd.P5 = lv_regs[15] << 8 | lv_regs[14];
		clv_cd.P6 = lv_regs[17] << 8 | lv_regs[16];
		clv_cd.P7 = lv_regs[19] << 8 | lv_regs[18];
		clv_cd.P8 = lv_regs[21] << 8 | lv_regs[20];
		clv_cd.P9 = lv_regs[23] << 8 | lv_regs[22];
	}
}

//============================================
//    public metods (funcs) for BMP280
//============================================
void cl_BMP280::begin() {	// defaults are 16x; Normal mode; 0.5ms, no filter, I2C
	begin(cd_FOR_MODE, cd_SB_500MS, cd_FIL_x16, cd_OS_x16, cd_OS_x16); // Forse mode, sleep 500ms, filter x16, t p h x16
  }

void cl_BMP280::begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p) {	// init bme280
	cl_BMP280::clf_readCalibCoef();

	uint8_t lv_reg_0xF4 = (osrs_t<<5) | (osrs_p<<2) | mode;
	uint8_t lv_reg_0xF5 = (t_sb << 5) | (filter << 2) | 0x00;
	
	cl_BMP280::writeReg(0xF4, lv_reg_0xF4);
	cl_BMP280::writeReg(0xF5, lv_reg_0xF5);
};    

tp_stru cl_BMP280::readTP(void) {
#define uint32_t uint32_t
	tp_stru lv_tp = { 0, 0 };
	int32_t  adc_T;
	uint32_t adc_P;
	uint8_t lv_nregs = 6;
	uint8_t lv_regs[lv_nregs];

	Wire.beginTransmission(clv_i2cAddr);   // addr of first byte raw data Humi
	Wire.write(0xF7);
	if (Wire.endTransmission() != 0) return lv_tp;	// something wrong with i2c connection and return 0

	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) 
		for (uint8_t i = 0; i < 6; i++) lv_regs[i] = Wire.read();
	adc_T = ((lv_regs[3] << 16) | (lv_regs[4] << 8) | lv_regs[5]) >> 4;
	adc_P = ((lv_regs[0] << 16) | (lv_regs[1] << 8) | lv_regs[2]) >> 4;

	// t_fine carries fine temperature as global value
	int32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tp.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		// adc_T >>= 4;
		lv_var1 = ((((adc_T >> 3) - ((int32_t)clv_cd.T1 << 1))) * ((int32_t)clv_cd.T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((int32_t)clv_cd.T1)) * ((adc_T >> 4) - ((int32_t)clv_cd.T1))) >> 12) * 
			((int32_t)clv_cd.T3)) >> 14;
		t_fine = lv_var1 + lv_var2;
		lv_tp.temp1 = ((float)((t_fine * 5 + 128) >> 8)) / 100;
	}

	int64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tp.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		// adc_P >>= 4;	// Start pressure converting
		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)clv_cd.P6;
		var2 = var2 + ((var1 * (int64_t)clv_cd.P5) << 17);
		var2 = var2 + (((int64_t)clv_cd.P4) << 35);
		var1 = ((var1 * var1 * (int64_t)clv_cd.P3) >> 8) + ((var1 * (int64_t)clv_cd.P2) << 12);
		var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)clv_cd.P1) >> 33;
		if (var1 == 0) {
			lv_tp.pres1 = 0;     // avoid exception caused by division by zero
		}
		else {
			p = 1048576 - adc_P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (((int64_t)clv_cd.P9) * (p >> 13) * (p >> 13)) >> 25;
			var2 = (((int64_t)clv_cd.P8) * p) >> 19;
			p = ((p + var1 + var2) >> 8) + (((int64_t)clv_cd.P7) << 4);
			lv_tp.pres1 = ((float)p) / 256;
		}
	}

	return lv_tp;
}



//============================================
//	specific private metods (funcs) for cl_BME280
//============================================
void cl_BME280::clf_readCalibCoef(void) {
	uint8_t lv_nregs = 26;
	uint8_t lv_regs[lv_nregs];		// temporary array for reading registers

	Wire.beginTransmission(clv_i2cAddr);   // first part request
	Wire.write(0x88);
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {    // reading 26 regs
		for (uint8_t i = 0; i < 25; i++) lv_regs[i] = Wire.read();

		clv_cd.T1 = lv_regs[1] << 8 | lv_regs[0];   // form struct
		clv_cd.T2 = lv_regs[3] << 8 | lv_regs[2];
		clv_cd.T3 = lv_regs[5] << 8 | lv_regs[4];
		clv_cd.P1 = lv_regs[7] << 8 | lv_regs[6];
		clv_cd.P2 = lv_regs[9] << 8 | lv_regs[8];
		clv_cd.P3 = lv_regs[11] << 8 | lv_regs[10];
		clv_cd.P4 = lv_regs[13] << 8 | lv_regs[12];
		clv_cd.P5 = lv_regs[15] << 8 | lv_regs[14];
		clv_cd.P6 = lv_regs[17] << 8 | lv_regs[16];
		clv_cd.P7 = lv_regs[19] << 8 | lv_regs[18];
		clv_cd.P8 = lv_regs[21] << 8 | lv_regs[20];
		clv_cd.P9 = lv_regs[23] << 8 | lv_regs[22];
		clv_cd.H1 = lv_regs[25];
	}

	Wire.beginTransmission(clv_i2cAddr);   // second part request
	Wire.write(0xE1);
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, 7) == 7) {   // reading
		for (uint8_t i = 0; i < 7; i++) lv_regs[i] = Wire.read();
		clv_cd.H2 = lv_regs[1] << 8 | lv_regs[0]; // (Wire.read() | (Wire.read() << 8));
		clv_cd.H3 = lv_regs[2];
		clv_cd.H4 = (((int16_t)lv_regs[3]) << 4) | (int16_t)(lv_regs[4] & 0x0F);
		clv_cd.H5 = (((int16_t)lv_regs[5]) << 4) | (int16_t)((lv_regs[4] & 0xF0) >> 4);
		clv_cd.H6 = lv_regs[6];
	};
}

//============================================
//    public metods (funcs) for cl_BME280
//============================================
void cl_BME280::begin() {	// defaults are 16x; Normal mode; 0.5ms, no filter, I2C
	begin(cd_FOR_MODE, cd_SB_500MS, cd_FIL_x16, cd_OS_x16, cd_OS_x16, cd_OS_x16); // Forse mode, sleep 500ms, filter x16, t p h x16
}

void cl_BME280::begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h) {	// init bme280
	cl_BME280::clf_readCalibCoef();

	uint8_t lv_reg_0xF2 = osrs_h;
	uint8_t lv_reg_0xF4 = (osrs_t<<5) | (osrs_p<<2) | mode;
	uint8_t lv_reg_0xF5 = (t_sb << 5) | (filter << 2) | 0x00;
	
	cl_BME280::writeReg(0xF2, lv_reg_0xF2);
	cl_BME280::writeReg(0xF4, lv_reg_0xF4);
	cl_BME280::writeReg(0xF5, lv_reg_0xF5);
};    

tph_stru cl_BME280::readTPH(void) {
	tph_stru lv_tph = { 0, 0, 0 };
	int32_t  adc_T;
	uint32_t adc_P;
	int32_t  adc_H;
	uint8_t lv_nregs = 8;
	uint8_t lv_regs[lv_nregs];

	Wire.beginTransmission(clv_i2cAddr);   // addr of first byte raw data Humi
	Wire.write(0xF7);
	if (Wire.endTransmission() != 0) return lv_tph;

	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs)
		for (uint8_t i = 0; i < 8; i++) lv_regs[i] = Wire.read();
	adc_T = ((lv_regs[3] << 16) | (lv_regs[4] << 8) | lv_regs[5]) >> 4;
	adc_P = ((lv_regs[0] << 16) | (lv_regs[1] << 8) | lv_regs[2]) >> 4;
	adc_H = (lv_regs[6] << 8) | lv_regs[7];

	// t_fine carries fine temperature as global value
	int32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tph.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		lv_var1 = ((((adc_T >> 3) - ((int32_t)clv_cd.T1 << 1))) * ((int32_t)clv_cd.T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((int32_t)clv_cd.T1)) * ((adc_T >> 4) - ((int32_t)clv_cd.T1))) >> 12) * 
			((int32_t)clv_cd.T3)) >> 14;
		t_fine = lv_var1 + lv_var2;
		lv_tph.temp1 = ((float)((t_fine * 5 + 128) >> 8)) / 100;
	}

	int64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tph.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		// adc_P >>= 4;	// Start pressure converting
		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)clv_cd.P6;
		var2 = var2 + ((var1 * (int64_t)clv_cd.P5) << 17);
		var2 = var2 + (((int64_t)clv_cd.P4) << 35);
		var1 = ((var1 * var1 * (int64_t)clv_cd.P3) >> 8) + ((var1 * (int64_t)clv_cd.P2) << 12);
		var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)clv_cd.P1) >> 33;
		if (var1 == 0) {
			lv_tph.pres1 = 0;     // avoid exception caused by division by zero
		}
		else {
			p = 1048576 - adc_P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (((int64_t)clv_cd.P9) * (p >> 13) * (p >> 13)) >> 25;
			var2 = (((int64_t)clv_cd.P8) * p) >> 19;
			p = ((p + var1 + var2) >> 8) + (((int64_t)clv_cd.P7) << 4);
			lv_tph.pres1 = ((float)p) / 256;
		}
	}

	int32_t v_x1_u32r;
	if (adc_H == 0x8000) {
		lv_tph.humi1 = 0;	// If the humidity module has been disabled return '0'
	}
	else {
		v_x1_u32r = t_fine - ((int32_t)76800);
		v_x1_u32r = (((((adc_H << 14) - (((int32_t)clv_cd.H4) << 20) - (((int32_t)clv_cd.H5) *
			v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
				((int32_t)clv_cd.H6)) >> 10) * (((v_x1_u32r * ((int32_t)clv_cd.H3)) >> 11) +
					((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)clv_cd.H2) + 8192) >> 14));
		v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)clv_cd.H1)) >> 4));
		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
		lv_tph.humi1 = (float)((uint32_t)(v_x1_u32r >> 12)) / 1024;
	}
	return lv_tph;
}



//============================================
//	specific private metods (funcs) for cl_BME680
//============================================
void cl_BME680::clf_readCalibCoef(void) {
	uint8_t lv_nregs = 23;
	uint8_t lv_regs[lv_nregs];		// temporary array for reading registers

	Wire.beginTransmission(clv_i2cAddr);	// first part request
	Wire.write(0x8A);						// Address of start calib. data (coeff.)
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {    // reading 26 regs
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();

		// par_t1 0xE9 / 0xEA
		// par_t2 0x8A / 0x8B
		// par_t3 0x8C
		clv_cd.T2 = ((int16_t)lv_regs[1] << 8) | lv_regs[0];	// fill struct
		clv_cd.T3 = lv_regs[2];
		clv_cd.P1 = lv_regs[5] << 8 | lv_regs[4];
		clv_cd.P2 = lv_regs[7] << 8 | lv_regs[6];
		clv_cd.P3 = lv_regs[8];
		clv_cd.P4 = lv_regs[11] << 8 | lv_regs[10];
		clv_cd.P5 = lv_regs[13] << 8 | lv_regs[12];
		clv_cd.P6 = lv_regs[15];
		clv_cd.P7 = lv_regs[14];
		clv_cd.P8 = lv_regs[19] << 8 | lv_regs[18];
		clv_cd.P9 = lv_regs[21] << 8 | lv_regs[20];
		clv_cd.P10 = lv_regs[22];
		// par_p1	0x8E / 0x8F
		// par_p2	0x90 / 0x91
		// par_p3	0x92
		// par_p4	0x94 / 0x95
		// par_p5	0x96 / 0x97
		// par_p6	0x99
		// par_p7	0x98
		// par_p8	0x9C / 0x9D
		// par_p9	0x9E / 0x9F
		// par_p10	0xA0
	}

	lv_nregs = 14;
	Wire.beginTransmission(clv_i2cAddr);	// second part request
	Wire.write(0xE1);						// Address of 2d part calibr data
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {   // reading
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();

		// par_h1 0xE2<3:0> / 0xE3
		// par_h2 0xE2<7:4> / 0xE1
		// par_h3 0xE4
		// par_h4 0xE5
		// par_h5 0xE6
		// par_h6 0xE7
		// par_h7 0xE8
		clv_cd.H2 = ((uint16_t)lv_regs[0] << 4) | ((uint16_t)lv_regs[1] >> 4);
		clv_cd.H1 = ((uint16_t)lv_regs[2] << 4) | (uint16_t)(lv_regs[1] & 0x0F);
		clv_cd.H3 = lv_regs[3];
		clv_cd.H4 = lv_regs[4];
		clv_cd.H5 = lv_regs[5];
		clv_cd.H6 = lv_regs[6];
		clv_cd.H7 = lv_regs[7];

		clv_cd.T1 = ((uint16_t)lv_regs[9] << 8) | lv_regs[8];
		
		// par_g1 0xED
		// par_g2 0xEB/0xEC
		// par_g3 0xEE
		// res_heat_range 0x02 <5:4>
		// res_heat_val 0x00
		clv_cd.G2 = ((int16_t)lv_regs[11] << 8) | (uint16_t)lv_regs[10];
		clv_cd.G1 = lv_regs[12];
		clv_cd.G3 = lv_regs[13];
	};
#ifdef enDEBUG
	printf("T1-T3  = %d %d %d \n", clv_cd.T1, clv_cd.T2, clv_cd.T3);
    printf("P1-p10 = %d %d %d %d %d %d %d %d %d %d \n", clv_cd.P1, clv_cd.P2, clv_cd.P3, clv_cd.P4, clv_cd.P5, clv_cd.P6, clv_cd.P7, clv_cd.P8, clv_cd.P9, clv_cd.P10);
    printf("H1-H7  = %d %d %d %d %d %d %d \n", clv_cd.H1, clv_cd.H2, clv_cd.H3, clv_cd.H4, clv_cd.H5, clv_cd.H6, clv_cd.H7);
    printf("G1-G3  = %d %d %d \n", clv_cd.G1, clv_cd.G2, clv_cd.G3);
#endif
}


//============================================
//    public metods (funcs) for cl_BME680
//============================================
void cl_BME680::do1Meas(void) {    // mode FORCED_MODE DO 1 Measuring
	cl_BME680::writeReg(0x74, cl_BME680::readReg(0x74) | 0X01);
}

bool cl_BME680::isMeas(void) {	// returns TRUE while bme680 is Measuring
	// Status reg 0x1D bit <6> gas measuring = 1 or bit <5> measuring = 1
	return (bool)((cl_BME680::readReg(0x1D) & 0x60));
}

void cl_BME680::begin() {	// defaults are 16x; Normal mode; 0.5ms, no filter, I2C
	cl_BME680::begin(cd_FIL_x16, cd_OS_x16, cd_OS_x16, cd_OS_x16); // filter x16, oversampling TPH x16
}

void cl_BME680::begin(uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h) {
	// Read calibration coefficients (data) to clas private (local) variable clv_cd
	cl_BME680::clf_readCalibCoef();
/*	Select mode, oversampling and filtering = Step 1, 2, 3
(osrs_h bit <2:0> regs 0x72, osrs_t bit <7:5> regs 0x74, osrs_p bit <4:2> regs 0x72, mode bit <1:0>)
p.16 of Bosch Document rev.: 1.9, Date: February 2024, Document N: BST-BME680-DS001-09
Technical reference code(s): 0 273 141 229; 0 273 141 312		*/
	cl_BME680::writeReg(0x72, osrs_h);
	cl_BME680::writeReg(0x74, ((osrs_t<<5) | (osrs_p<<2) | 0x00) );
};    

void cl_BME680::initGasPointX(uint8_t lp_setPoint, uint16_t lp_tagTemp, uint16_t lp_duration, int16_t lp_ambTemp) {
	//  Up to 10 different hot plate temperature set points can be configured by setting the registers res_heat_x<7:0>,
	//  where x = 0…9. The internal heater control loop operates on the resistance of the heater structure.
	//  Hence, the user first needs to convert the target temperature into a device specific target resistance
	//  before writing the resulting register code into the sensor memory map.

	//	Enable GAS conversion = Step 4. run_gas =1 (bit <4> reg 0x71) and 
	//	Select index of heater set-point 0 = Step 5. nb_conv = 0 (bit 3:0 reg 0x71)
	if (lp_setPoint > 9) lp_setPoint = 9;
	cl_BME680::writeReg(0x71, (0x10 | lp_setPoint) );

	//	Define heater duration Temp = Step 6. gas_wait_0 = 63 ms (bit 5:0 ms and bit 7:6 multiplier reg 0x64-0x6D)
	if (lp_duration >= 0x0FC0) {
		lp_duration = 0x00FF;
	}
	else {
		uint8_t lv_mult = 0;
		while (lp_duration > 63) {
			lv_mult++;
			lp_duration = lp_duration >> 2;
		}
		lp_duration = lp_duration | (lv_mult << 6);
	}
	cl_BME680::writeReg(0x64 + lp_setPoint, (uint8_t)lp_duration);

	//	Set heater Temp = Step 7. Convert temperature to register cspecific val. Set res_heat_0 (bit <7:0> reg 0x5A-0x63)
	//	make function clf_calcResHeatX()

	// •	par_g1, par_g2, and par_g3 are calibration parameters,
	// •	target_temp is the target heater temperature in degree Celsius,
	// •	amb_temp is the ambient temperature (hardcoded or read from temperature sensor),
	// •	var5 is the target heater resistance in Ohm,
	// •	res_heat_x is the decimal value that needs to be stored in register,
	// 			where 'x' corresponds to the temperature profilenumber between 0 and 9,
	// •	res_heat_range is the heater range stored in register address 0x02 <5:4>, and
	// •	res_heat_val is the heater resistance correction factor stored in register address 0x00 
	// 			(signed, value from -128 to 127).
	int32_t var1 = (((int32_t)lp_ambTemp * clv_cd.G3) / 10) << 8;
	int32_t var2 = (clv_cd.G1 + 784) * (((((clv_cd.G2 + 154009) * lp_tagTemp * 5) / 100) + 3276800) / 10);
	int32_t var3 = var1 + (var2 >> 1);
	int8_t  res_heat_val   = cl_BME680::readReg(0x00);
	uint8_t res_heat_range = (cl_BME680::readReg(0x02) >> 4) & 0x03;
	int32_t var4 = (var3 / (res_heat_range + 4));
	int32_t var5 = (131 * res_heat_val) + 65536;
	int32_t res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
	uint8_t res_heat_0 = (uint8_t)((res_heat_x100 + 50) / 100);
	cl_BME680::writeReg(0x5A + lp_setPoint, res_heat_0);
}

tphg_stru cl_BME680::readTPHG(void) {
	tphg_stru lv_tphg = { 0, 0, 0, 0 };
	uint32_t  adc_T;
	uint32_t  adc_P;
	uint32_t  adc_H;
	uint32_t  adc_G;
	//	Construct normal value from raw data
	uint8_t lv_nregs = 13;					//	read 14 bytes raw data form 0x1F to 0x1B at once
	uint8_t lv_regs[lv_nregs];
	Wire.beginTransmission(clv_i2cAddr);	// addr of start byte raw data (adc) P T H
	Wire.write(0x1F);
	if (Wire.endTransmission() != 0) return lv_tphg;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs)
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();

	adc_P = (uint32_t)0 | (lv_regs[0] << 12) | (lv_regs[1] << 4) | (lv_regs[2] >> 4);
	adc_T = (uint32_t)0 | (lv_regs[3] << 12) | (lv_regs[4] << 4) | (lv_regs[5] >> 4);
	adc_H = (uint32_t)0 | (lv_regs[6] << 8) | lv_regs[7];
	adc_G = (uint32_t)0 | ((uint32_t)lv_regs[11] << 2) | (uint32_t)(lv_regs[12] >> 6);
	uint8_t gas_range = lv_regs[12] & 0x0F;
	uint8_t range_switching_error = (cl_BME680::readReg(0x04) >> 4);
	if ( !(lv_regs[12] & 0b00100000) ) Serial.println("Gas Valid = 0 !!!");		// Test for Ok gas measuring
	if ( !(lv_regs[12] & 0b00010000) ) Serial.println("Heat Stable = 0 !!!");	// Test for Ok gas preheating

	// Calc T, where
	// par_t1, par_t2 and par_t3 are calibration parameters,
	// adc_T is the raw temperature output data,
	// t_fine carries fine temperature that will use in future calc
	// temp_comp is the compensated temperature output data in degrees Celsius.
	int32_t lv_var1, lv_var2, lv_var3, t_fine, temp_comp;
	if (adc_T == 0x800000) lv_tphg.temp1 = 0;	// if the temperature module has been disabled return '0'
	else {
		lv_var1 = ((int32_t)adc_T >> 3) - ((int32_t)clv_cd.T1 << 1);
		lv_var2 = (lv_var1 * (int32_t)clv_cd.T2) >> 11;
		lv_var3 = ((((lv_var1 >> 1) * (lv_var1 >> 1)) >> 12) * ((int32_t)clv_cd.T3 << 4)) >> 14;
		t_fine = lv_var2 + lv_var3;
		temp_comp = ((t_fine * 5) + 128) >> 8;
		lv_tphg.temp1 = ((float)temp_comp) / 100;

		// Calc P, where
		// • par_p1, par_p2, …, par_p10 are calibration parameters,
		// • press_adc is the raw pressure output data,
		// • press_comp is the compensated pressure output data in Pascal.
		uint32_t press_comp;
		if (adc_P == 0x800000) {
			lv_tphg.pres1 = 0;	// If the pressure module has been disabled return '0'
		}
		else {
			lv_var1 = ((int32_t)t_fine >> 1) - 64000;
			lv_var2 = ((((lv_var1 >> 2) * (lv_var1 >> 2)) >> 11) * (int32_t)clv_cd.P6) >> 2;
			lv_var2 = lv_var2 + ((lv_var1 * (int32_t)clv_cd.P5) << 1);
			lv_var2 = (lv_var2 >> 2) + ((int32_t)clv_cd.P4 << 16);
			lv_var1 = (((((lv_var1 >> 2) * (lv_var1 >> 2)) >> 13) * ((int32_t)clv_cd.P3 << 5)) >> 3) + (((int32_t)clv_cd.P2 * lv_var1) >> 1);
			lv_var1 = lv_var1 >> 18;
			lv_var1 = ((32768 + lv_var1) * (int32_t)clv_cd.P1) >> 15;
			press_comp = 1048576 - adc_P;
			press_comp = (uint32_t)((press_comp - (lv_var2 >> 12)) * ((uint32_t)3125));
			if (press_comp >= (1 << 30))
				press_comp = ((press_comp / (uint32_t)lv_var1) << 1);
			else
				press_comp = ((press_comp << 1) / (uint32_t)lv_var1);
			lv_var1 = ((int32_t)clv_cd.P9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
			lv_var2 = ((int32_t)(press_comp >> 2) * (int32_t)clv_cd.P8) >> 13;
			lv_var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)clv_cd.P10) >> 17;
			press_comp = (int32_t)(press_comp)+((lv_var1 + lv_var2 + lv_var3 + ((int32_t)clv_cd.P7 << 7)) >> 4);
			lv_tphg.pres1 = (float)press_comp;
		}

		// Calc H, where
		// • par_h1, par_h2, …, par_h7 are calibration parameters,
		// • hum_adc is the raw humidity output data,
		// • hum_comp is the compensated humidity output data in percent.
		int32_t lv_var4, lv_var5, lv_var6, hum_comp;
		if (adc_H == 0x8000) lv_tphg.humi1 = 0;	// If the humidity module has been disabled return '0'
		else {
			int32_t temp_scaled = (int32_t)temp_comp;
			lv_var1 = (int32_t)adc_H - (int32_t)((int32_t)clv_cd.H1 << 4) -
				(((temp_scaled * (int32_t)clv_cd.H3) / ((int32_t)100)) >> 1);
			lv_var2 = ((int32_t)clv_cd.H2 * (((temp_scaled *
				(int32_t)clv_cd.H4) / ((int32_t)100)) +
				(((temp_scaled * ((temp_scaled * (int32_t)clv_cd.H5) /
					((int32_t)100))) >> 6) / ((int32_t)100)) + ((int32_t)(1 << 14)))) >> 10;
			lv_var3 = lv_var1 * lv_var2;
			lv_var4 = (((int32_t)clv_cd.H6 << 7) +
				((temp_scaled * (int32_t)clv_cd.H7) / ((int32_t)100))) >> 4;
			lv_var5 = ((lv_var3 >> 14) * (lv_var3 >> 14)) >> 10;
			lv_var6 = (lv_var4 * lv_var5) >> 1;
			hum_comp = (((lv_var3 + lv_var6) >> 10) * ((int32_t)1000)) >> 12;
			lv_tphg.humi1 = ((float)hum_comp) / 1024;	// ???
		}

		// Calc Gas resistance,
		// Readout of gas sensor resistance ADC value and calculation of gas sensor resistance consists of 3 steps
		// 1. Read gas ADC value (gas_adc) and gas ADC range (gas_range) (see Section 5.3.4)
		// 2. Read range switching error from register address 0x04 <7:4> (signed 4 bit)
		// 3. Convert ADC value into gas sensor resistance in ohm
		if (adc_G == 0x8000) lv_tphg.gasr1 = 0;	// If the humidity module has been disabled return '0'
		else {
			const uint32_t uintTab1[16] = {
			UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
			UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
			UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
			UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };

			const uint32_t uintTab2[16] = {
			UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
			UINT32_C(255744255),  UINT32_C(127110228),  UINT32_C(64000000),   UINT32_C(32258064),
			UINT32_C(16016016),   UINT32_C(8000000),    UINT32_C(4000000),    UINT32_C(2000000),
			UINT32_C(1000000),    UINT32_C(500000),     UINT32_C(250000),     UINT32_C(125000) };

			// 0x2A<9:2>/0x2B<7:6>=<1:0>	gas_adc is the raw gas sensor resistance output data (i.e. ADC value),
			// 0x2B<3:0>				gas_range 	is the ADC range of the measured gas sensor resistance,
			// 0x04<7:4>	range_switching_error 	is a calibration parameter,
			// 							gas_res 	is the compensated gas sensor resistance output data in Ohms.
			int64_t		var1, var3;
			uint64_t	var2;
			var1 = (int64_t)((1340 + (5 * (int64_t)range_switching_error)) * ((int64_t)uintTab1[gas_range])) >> 16;
			var2 = (((int64_t)((int64_t)adc_G << 15) - (int64_t)(16777216)) + var1);
			var3 = (((int64_t)uintTab2[gas_range] * (int64_t)var1) >> 9);
			uint32_t gas_res = (uint32_t)((var3 + ((int64_t)var2 >> 1)) / (int64_t)var2);
			lv_tphg.gasr1 = ((float)gas_res)/1000;
		}

		return lv_tphg;
	}
}
//============================================================================================================