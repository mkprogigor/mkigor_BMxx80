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

		clv_cd.dig_T1 = lv_regs[1] << 8 | lv_regs[0];   // form struct
		clv_cd.dig_T2 = lv_regs[3] << 8 | lv_regs[2];
		clv_cd.dig_T3 = lv_regs[5] << 8 | lv_regs[4];
		clv_cd.dig_P1 = lv_regs[7] << 8 | lv_regs[6];
		clv_cd.dig_P2 = lv_regs[9] << 8 | lv_regs[8];
		clv_cd.dig_P3 = lv_regs[11] << 8 | lv_regs[10];
		clv_cd.dig_P4 = lv_regs[13] << 8 | lv_regs[12];
		clv_cd.dig_P5 = lv_regs[15] << 8 | lv_regs[14];
		clv_cd.dig_P6 = lv_regs[17] << 8 | lv_regs[16];
		clv_cd.dig_P7 = lv_regs[19] << 8 | lv_regs[18];
		clv_cd.dig_P8 = lv_regs[21] << 8 | lv_regs[20];
		clv_cd.dig_P9 = lv_regs[23] << 8 | lv_regs[22];
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
#define BME280_S32_t int32_t		// make compatible code to BOSCH datasheet example
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t
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
	BME280_S32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tp.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		// adc_T >>= 4;
		lv_var1 = ((((adc_T >> 3) - ((BME280_S32_t)clv_cd.dig_T1 << 1))) * ((BME280_S32_t)clv_cd.dig_T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((BME280_S32_t)clv_cd.dig_T1)) * ((adc_T >> 4) - ((BME280_S32_t)clv_cd.dig_T1))) >> 12) * ((BME280_S32_t)clv_cd.dig_T3)) >> 14;
		t_fine = lv_var1 + lv_var2;
		lv_tp.temp1 = ((float)((t_fine * 5 + 128) >> 8)) / 100;
	}

	BME280_S64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tp.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		// adc_P >>= 4;	// Start pressure converting
		var1 = ((BME280_S64_t)t_fine) - 128000;
		var2 = var1 * var1 * (BME280_S64_t)clv_cd.dig_P6;
		var2 = var2 + ((var1 * (BME280_S64_t)clv_cd.dig_P5) << 17);
		var2 = var2 + (((BME280_S64_t)clv_cd.dig_P4) << 35);
		var1 = ((var1 * var1 * (BME280_S64_t)clv_cd.dig_P3) >> 8) + ((var1 * (BME280_S64_t)clv_cd.dig_P2) << 12);
		var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)clv_cd.dig_P1) >> 33;
		if (var1 == 0) {
			lv_tp.pres1 = 0;     // avoid exception caused by division by zero
		}
		else {
			p = 1048576 - adc_P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (((BME280_S64_t)clv_cd.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
			var2 = (((BME280_S64_t)clv_cd.dig_P8) * p) >> 19;
			p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)clv_cd.dig_P7) << 4);
			lv_tp.pres1 = ((float)((BME280_U32_t)((p * 100) / 256))) / 100;
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

		clv_cd.dig_T1 = lv_regs[1] << 8 | lv_regs[0];   // form struct
		clv_cd.dig_T2 = lv_regs[3] << 8 | lv_regs[2];
		clv_cd.dig_T3 = lv_regs[5] << 8 | lv_regs[4];
		clv_cd.dig_P1 = lv_regs[7] << 8 | lv_regs[6];
		clv_cd.dig_P2 = lv_regs[9] << 8 | lv_regs[8];
		clv_cd.dig_P3 = lv_regs[11] << 8 | lv_regs[10];
		clv_cd.dig_P4 = lv_regs[13] << 8 | lv_regs[12];
		clv_cd.dig_P5 = lv_regs[15] << 8 | lv_regs[14];
		clv_cd.dig_P6 = lv_regs[17] << 8 | lv_regs[16];
		clv_cd.dig_P7 = lv_regs[19] << 8 | lv_regs[18];
		clv_cd.dig_P8 = lv_regs[21] << 8 | lv_regs[20];
		clv_cd.dig_P9 = lv_regs[23] << 8 | lv_regs[22];
		clv_cd.dig_H1 = lv_regs[25];
	}

	Wire.beginTransmission(clv_i2cAddr);   // second part request
	Wire.write(0xE1);
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, 7) == 7) {   // reading
		for (uint8_t i = 0; i < 7; i++) lv_regs[i] = Wire.read();
		clv_cd.dig_H2 = lv_regs[1] << 8 | lv_regs[0]; // (Wire.read() | (Wire.read() << 8));
		clv_cd.dig_H3 = lv_regs[2];
		clv_cd.dig_H4 = ((int16_t)lv_regs[3] << 4) | (lv_regs[4] & 0x0F);
		clv_cd.dig_H5 = ((int16_t)lv_regs[5] << 4) | ((lv_regs[4] & 0xF0) >> 4);
		clv_cd.dig_H6 = lv_regs[6];
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
#define BME280_S32_t int32_t		// make compatible code to BOSCH datasheet example
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t
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
	BME280_S32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tph.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		// adc_T >>= 4;
		lv_var1 = ((((adc_T >> 3) - ((BME280_S32_t)clv_cd.dig_T1 << 1))) * ((BME280_S32_t)clv_cd.dig_T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((BME280_S32_t)clv_cd.dig_T1)) * ((adc_T >> 4) - ((BME280_S32_t)clv_cd.dig_T1))) >> 12) * ((BME280_S32_t)clv_cd.dig_T3)) >> 14;
		t_fine = lv_var1 + lv_var2;
		lv_tph.temp1 = ((float)((t_fine * 5 + 128) >> 8)) / 100;
	}

	BME280_S64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tph.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		// adc_P >>= 4;	// Start pressure converting
		var1 = ((BME280_S64_t)t_fine) - 128000;
		var2 = var1 * var1 * (BME280_S64_t)clv_cd.dig_P6;
		var2 = var2 + ((var1 * (BME280_S64_t)clv_cd.dig_P5) << 17);
		var2 = var2 + (((BME280_S64_t)clv_cd.dig_P4) << 35);
		var1 = ((var1 * var1 * (BME280_S64_t)clv_cd.dig_P3) >> 8) + ((var1 * (BME280_S64_t)clv_cd.dig_P2) << 12);
		var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)clv_cd.dig_P1) >> 33;
		if (var1 == 0) {
			lv_tph.pres1 = 0;     // avoid exception caused by division by zero
		}
		else {
			p = 1048576 - adc_P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (((BME280_S64_t)clv_cd.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
			var2 = (((BME280_S64_t)clv_cd.dig_P8) * p) >> 19;
			p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)clv_cd.dig_P7) << 4);
			lv_tph.pres1 = ((float)((BME280_U32_t)((p * 100) / 256))) / 100;
		}
	}

	BME280_S32_t v_x1_u32r;
	if (adc_H == 0x8000) {
		lv_tph.humi1 = 0;	// If the humidity module has been disabled return '0'
	}
	else {
		v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
		v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)clv_cd.dig_H4) << 20) - (((BME280_S32_t)clv_cd.dig_H5) *
			v_x1_u32r)) + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *
				((BME280_S32_t)clv_cd.dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)clv_cd.dig_H3)) >> 11) +
					((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)clv_cd.dig_H2) + 8192) >> 14));
		v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)clv_cd.dig_H1)) >> 4));
		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
		lv_tph.humi1 = ((float)((BME280_U32_t)(((v_x1_u32r >> 12) * 1000) / 1024))) / 1000;
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

		clv_cd.T2 = lv_regs[1] << 8 | lv_regs[0];	// fill struct
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
	}

	lv_nregs = 14;
	Wire.beginTransmission(clv_i2cAddr);	// second part request
	Wire.write(0xE1);						// Address of 2d part calibr data
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {   // reading
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();

		clv_cd.H2 = ((lv_regs[0] ) << 12) | (lv_regs[1] >> 4);
		clv_cd.H2 = ((lv_regs[2] ) << 12) | (lv_regs[1] & 0x0F);
		clv_cd.H3 = lv_regs[3];
		clv_cd.H4 = lv_regs[4];
		clv_cd.H5 = lv_regs[5];
		clv_cd.H6 = lv_regs[6];
		clv_cd.H7 = lv_regs[7];

		clv_cd.T1 = lv_regs[9] << 8 | lv_regs[8];
		
		clv_cd.G2 = lv_regs[11] << 8 | lv_regs[10];
		clv_cd.G1 = lv_regs[12];
		clv_cd.G3 = lv_regs[13];
	};
}

uint8_t cl_BME680::clf_calcResHeatX(int16_t lp_ambTemp, int16_t lp_tagTemp) {
	int32_t lp_ambTemp = 20;
	int32_t var1 = (((int32_t)lp_ambTemp * clv_cd.G3) / 10) << 8;
	int32_t var2 = (clv_cd.G1 + 784) * (((((clv_cd.G2 + 154009) * lp_tagTemp * 5) / 100) + 3276800) / 10);
	int32_t var3 = var1 + (var2 >> 1);
	int32_t var4 = (var3 / (res_heat_range + 4));
	int8_t  res_heat_val   = cl_BME680::readReg(0x00);
	uint8_t res_heat_range = (cl_BME680::readReg(0x02) >> 4) & 0x03;
	int32_t var5 = (131 * res_heat_val) + 65536;
	int32_t res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
	uint8_t res_heat_x = (uint8_t)((res_heat_x100 + 50) / 100);
	return res_heat_x
}


//============================================
//    public metods (funcs) for cl_BME680
//============================================
void cl_BME680::do1Meas(void) {    // mode FORCED_MODE DO 1 Measuring
	uint8_t lv_treg = cl_BME680::readReg(0x74);
	lv_treg = (lv_treg & 0xFC) | 0x01;
	cl_BME680::writeReg(0x74, lv_treg);
}

bool cl_BME680::isMeas(void) {	// returns TRUE while bme680 is Measuring
	lv_tregs1 =	cl_BME680::readReg(0x1D);					
	lv_tregs2 =	cl_BME680::readReg(0x2B);
//	print status bits
	if (lv_tregs1 & 0x80 >>7) Serial.println("Status reg 0x1D bit <7> new_data_0 = 1");
	if (lv_tregs1 & 0x40 >>6) Serial.println("Status reg 0x1D bit <6> gas_measuring = 1");
	if (lv_tregs1 & 0x20 >>5) Serial.println("Status reg 0x1D bit <5> measuring = 1");

	if (lv_tregs2 & 0x20 >>5) Serial.println("Status Gas reg 0x2B bit <5> gas_valid_r = 1");
	if (lv_tregs2 & 0x10 >>4) Serial.println("Status Gas reg 0x2B bit <4> heat_stab_r = 1");
	
	return (bool)((clv_tregs1 & 0x20) >> 5);  	 // Status reg 0x1D bit <5> measuring = 1
}

void cl_BME680::begin() {	// defaults are 16x; Normal mode; 0.5ms, no filter, I2C
	cl_BME680::begin(cd_FIL_x16, cd_OS_x16, cd_OS_x16, cd_OS_x16); // filter x16, oversampling TPH x16
}

void cl_BME680::begin(uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, int16_t lp_tagTemp) {	// init bme280
// Read calibration coefficients (data) to clas private (local) variable clv_cd
	cl_BME680::clf_readCalibCoef();

/*	Select mode, oversampling and filtering = Step 1, 2, 3
(osrs_h bit <2:0> regs 0x72, osrs_t bit <7:5> regs 0x74, osrs_p bit <4:2> regs 0x72, mode bit <1:0>)
p.16 of Bosch Document rev.: 1.9, Date: February 2024, Document N: BST-BME680-DS001-09
Technical reference code(s): 0 273 141 229; 0 273 141 312		*/
	cl_BME680::writeReg(0x72, osrs_h);
	cl_BME680::writeReg(0x74, ((osrs_t<<5) | (osrs_p<<2) | 0x00) );

	//	Enable GAS conversion = Step 4 run_gas =1 (bit <4> reg 0x71) and 
	//	Select index of heater set-point 0 = Step 5 nb_conv = 0 (bit 3:0 reg 0x71)
	cl_BME680::writeReg(0x71, 0b00010000);

	//	Define heater on Time Temp = Step 6. gas_wait_0 = 63 ms (bit 5:0 ms and bit 7:6 multiplier reg 0x6D)
	cl_BME680::writeReg(0x6D, 0b00011111);

	//	Set heater Temp = Step 7. Convert temperature to register code. Set res_heat_0 (bit <7:0> reg 0x63)
	//	make function clf_calcResHeatX()
	uint8_t res_heat_x = cl_BME680::clf_calcResHeatX(20, lp_tagTemp);
	cl_BME680::writeReg(0x6D, res_heat_x);

	//	Set mode to forced mode (let do measure TPHG => fn Do1Meas() ) = Step 8. Set mode = 0b01 (bit <1:0> reg 0x74)
	//	fn cl_BME680::do1Meas(void)
};    

tphg_stru cl_BME680::readTPHG(void) {
#define BME280_S32_t int32_t		// make compatible code to BOSCH datasheet example
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t
	tphg_stru lv_tphg = { 0, 0, 0, 0 };
	int32_t  adc_T;
	uint32_t adc_P;
	int32_t  adc_H;
	uint32_t adc_G;
	uint8_t lv_nregs = 8;
	uint8_t lv_regs[lv_nregs];

	Wire.beginTransmission(clv_i2cAddr);   // addr of first byte raw data Humi
	Wire.write(0xF7);
	if (Wire.endTransmission() != 0) return lv_tphg;

	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs)
		for (uint8_t i = 0; i < 8; i++) lv_regs[i] = Wire.read();
	adc_T = ((lv_regs[3] << 16) | (lv_regs[4] << 8) | lv_regs[5]) >> 4;
	adc_P = ((lv_regs[0] << 16) | (lv_regs[1] << 8) | lv_regs[2]) >> 4;
	adc_H = (lv_regs[6] << 8) | lv_regs[7];

	// t_fine carries fine temperature as global value
	BME280_S32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tphg.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		// adc_T >>= 4;
		lv_var1 = ((((adc_T >> 3) - ((BME280_S32_t)clv_cd.dig_T1 << 1))) * ((BME280_S32_t)clv_cd.dig_T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((BME280_S32_t)clv_cd.dig_T1)) * ((adc_T >> 4) - ((BME280_S32_t)clv_cd.dig_T1))) >> 12) * ((BME280_S32_t)clv_cd.dig_T3)) >> 14;
		t_fine = lv_var1 + lv_var2;
		lv_tphg.temp1 = ((float)((t_fine * 5 + 128) >> 8)) / 100;
	}

	BME280_S64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tphg.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		// adc_P >>= 4;	// Start pressure converting
		var1 = ((BME280_S64_t)t_fine) - 128000;
		var2 = var1 * var1 * (BME280_S64_t)clv_cd.dig_P6;
		var2 = var2 + ((var1 * (BME280_S64_t)clv_cd.dig_P5) << 17);
		var2 = var2 + (((BME280_S64_t)clv_cd.dig_P4) << 35);
		var1 = ((var1 * var1 * (BME280_S64_t)clv_cd.dig_P3) >> 8) + ((var1 * (BME280_S64_t)clv_cd.dig_P2) << 12);
		var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)clv_cd.dig_P1) >> 33;
		if (var1 == 0) {
			lv_tphg.pres1 = 0;     // avoid exception caused by division by zero
		}
		else {
			p = 1048576 - adc_P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (((BME280_S64_t)clv_cd.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
			var2 = (((BME280_S64_t)clv_cd.dig_P8) * p) >> 19;
			p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)clv_cd.dig_P7) << 4);
			lv_tphg.pres1 = ((float)((BME280_U32_t)((p * 100) / 256))) / 100;
		}
	}

	BME280_S32_t v_x1_u32r;
	if (adc_H == 0x8000) {
		lv_tphg.humi1 = 0;	// If the humidity module has been disabled return '0'
	}
	else {
		v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
		v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)clv_cd.dig_H4) << 20) - (((BME280_S32_t)clv_cd.dig_H5) *
			v_x1_u32r)) + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *
				((BME280_S32_t)clv_cd.dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)clv_cd.dig_H3)) >> 11) +
					((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)clv_cd.dig_H2) + 8192) >> 14));
		v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)clv_cd.dig_H1)) >> 4));
		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
		lv_tphg.humi1 = ((float)((BME280_U32_t)(((v_x1_u32r >> 12) * 1000) / 1024))) / 1000;
	}
	return lv_tphg;
}

//============================================================================================================