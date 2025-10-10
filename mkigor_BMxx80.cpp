/*
	@brief		C++ library Arduino framework for Bosch sensors: BMP280, BME280, BME680, via i2c.
	@author		Igor Mkprog, mkprogigor@gmail.com
	@version	V1.1	@date	10.10.2025

	@remarks	Glossary, abbreviations used in the module.
	@details	Name of functions dont use symbol '_', only small or capital letters.
				Symbol '_' divide name in: prefix _ name _ suffix:
	@var		gv_*    -   Global Variable;
				lv_*    -   Local Variable (live inside statement);
				cl_*    -   CLass;
				cd_*    -   Class Definishion;
				cgv_*   -   Class public (Global) member (Variable);
				clv_*   -   Class private (Local) member (Variable);
				cgf_*   -   Class public (Global) metod (Function), not need, no usefull, becouse we see parenthesis => ();
				clf_*   -   Class private (Local) metod (Function);
				lp_		-	in function, local parameter
				*_stru  -   [or *_stru_t] suffix, as usual, point the type.
*/

#include <mkigor_BMxx80.h>
#include <Arduino.h>

// #define enDEBUG		//	if need addition print info, uncomment this string

//============================================
//	BMP280, BME280, BME680
//	cl_BMP280, cl_BME280, cl_BME680 common public metod (function)
//============================================
/*	@brief	Read 1 byte from register with address,
	@param	address is address of register to read
	@return	1 byteb read or 0 if operation not success	*/
uint8_t cl_BMP280::readReg(uint8_t address) {
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(address);
	if (Wire.endTransmission() != 0) return 0;
	if (Wire.requestFrom(clv_i2cAddr, 1) == 1) return Wire.read();
	else return 0;
}

/*	@brief	Write 1 byte to register with address,
	@param	address is address of register to write
	@param	data is byte to write	
	@return	TRUE if operation is success, otherwise FALSE	*/
bool cl_BMP280::writeReg(uint8_t address, uint8_t data) {
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(address);
	Wire.write(data);
	if (Wire.endTransmission() == 0) return true;
	else return false;
}

/*	@brief	Check conection with sensor,
	fn return chip codes: 0x58=BMP280, 0x60=BME280, 0x61=BME680.
	i2c address 0x76, 0x77 possible for BMP280 or BME280 or BME680, note: CHECK IT ! 
	@return	Chip_code is senor is present, if NO return 0	*/
uint8_t cl_BMP280::check(uint8_t lv_i2caddr) {
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
}

/*	@brief	Software reset of bmp280, bme280, bme680. 
	@return TRUE if write operation is OK	*/
bool cl_BMP280::reset(void) {
	if (cl_BMP280::writeReg(0x0E, 0xB6)) return true;
    else return false;
}

/*	@brief	Send to sensor command Start Measuring (in FORCED mode)	*/
void cl_BMP280::do1Meas(void) {
	uint8_t lv_reg_0xF4 = cl_BMP280::readReg(0xF4);
	cl_BMP280::writeReg(0xF4, ((lv_reg_0xF4 & 0xFC) | 0x01));
}

/*	@brief Test if sensor is Measuring 
	@return TRUE while bmp280 is Measuring of FALSE when it is sleep	*/
bool cl_BMP280::isMeas(void) {								
	return (bool)((cl_BMP280::readReg(0xF3) & 0x08) >> 3);
}



//============================================
//	BMP280
//	cl_BMP280, specific private metods (funcs)
//============================================
/*	@brief	Read Calibration Data for BMP280 in clv_cd var structure	*/
void cl_BMP280::clf_readCalibData(void) {
	uint8_t lv_nregs = 24;
	uint8_t lv_regs[lv_nregs];		// temporary array for reading registers

	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(0x88);
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {    // reading 24 regs
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();
		clv_cd.T1 = lv_regs[1] << 8 | lv_regs[0];
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
//	cl_BMP280, public metods (funcs)
//============================================
/*	@brief Read calibration data and Init sensor with default
	force mode, filter value: cd_FIL_x2, stand by time 500ms, oversampling value T P : cd_OS_x16	*/
void cl_BMP280::begin() {
	begin(cd_FOR_MODE, cd_SB_500MS, cd_FIL_x16, cd_OS_x16, cd_OS_x16);
  }

/*	@brief Read calibration data and Init sensor with
	@param mode		cd_FOR_MODE or cd_NOR_MODE
	@param t_sb		time standby in cd_NOR_MODE
	@param filter	filter value: cd_FIL_OFF .. cd_FIL_x128
	@param osrs_t	oversampling value temperature: cd_OS_OFF..cd_OS_x16
	@param osrs_p	oversampling value pressure: cd_OS_OFF..cd_OS_x16	*/
void cl_BMP280::begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p) {	// init bme280
	cl_BMP280::clf_readCalibData();
	uint8_t lv_reg_0xF4 = (osrs_t<<5) | (osrs_p<<2) | mode;
	uint8_t lv_reg_0xF5 = (t_sb << 5) | (filter << 2) | 0x00;
	cl_BMP280::writeReg(0xF4, lv_reg_0xF4);
	cl_BMP280::writeReg(0xF5, lv_reg_0xF5);
};    

/*	@brief Read raw data (adc_ P T) & calc it to compensate value
	@returns compensate value of T P in structure var		*/
tp_stru cl_BMP280::readTP(void) {
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

	int32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tp.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		lv_var1 = ((((adc_T >> 3) - ((int32_t)clv_cd.T1 << 1))) * ((int32_t)clv_cd.T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((int32_t)clv_cd.T1)) * ((adc_T >> 4) - ((int32_t)clv_cd.T1))) >> 12) * 
			((int32_t)clv_cd.T3)) >> 14;
		t_fine = lv_var1 + lv_var2;		// t_fine carries fine temperature as global value
		lv_tp.temp1 = ((float)((t_fine * 5 + 128) >> 8)) / 100;
	}

	int64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tp.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
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
//	BME280,
//	cl_BME280, specific private metods (funcs)
//============================================
/*	@brief	Read Calibration Data for BME280 in clv_cd var structure	*/
void cl_BME280::clf_readCalibData(void) {
	uint8_t lv_nregs = 26;
	uint8_t lv_regs[lv_nregs];		// temporary array for reading registers

	Wire.beginTransmission(clv_i2cAddr);   // first part request
	Wire.write(0x88);
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {    // reading 26 regs
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();
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

	lv_nregs = 7;
	Wire.beginTransmission(clv_i2cAddr);   // second part request 7 regs
	Wire.write(0xE1);
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();
		clv_cd.H2 = lv_regs[1] << 8 | lv_regs[0];
		clv_cd.H3 = lv_regs[2];
		clv_cd.H4 = ( ( (int16_t)(int8_t)lv_regs[3] ) * 16) | (int16_t)(lv_regs[4] & 0x0F );
		clv_cd.H5 = ( ( (int16_t)(int8_t)lv_regs[5] ) * 16) | (int16_t)(lv_regs[4]  >> 4  );
		clv_cd.H6 = lv_regs[6];
	};
#ifdef enDEBUG
	printf("\nCalibrated data BME280:\n", clv_cd.T1, clv_cd.T2, clv_cd.T3);
	printf("T1-T3 = %d %d %d \n", clv_cd.T1, clv_cd.T2, clv_cd.T3);
    printf("P1-p9 = %d %d %d %d %d %d %d %d %d \n", clv_cd.P1, clv_cd.P2, clv_cd.P3, clv_cd.P4, clv_cd.P5, clv_cd.P6, clv_cd.P7, clv_cd.P8, clv_cd.P9);
    printf("H1-H6 = %d %d %d %d %d %d \n\n", clv_cd.H1, clv_cd.H2, clv_cd.H3, clv_cd.H4, clv_cd.H5, clv_cd.H6);
#endif
}

//============================================
//	cl_BME280, public metods (funcs)
//============================================
/*	@brief Read calibration data and Init sensor with default
	force mode, filter value: cd_FIL_x2, stand by time 500ms, oversampling value T P H : cd_OS_x16	*/
void cl_BME280::begin() {
	begin(cd_FOR_MODE, cd_SB_500MS, cd_FIL_x16, cd_OS_x16, cd_OS_x16, cd_OS_x16); // Forse mode, sleep 500ms, filter x16, t p h x16
}

/*	@brief Read calibration data and Init sensor with
	@param mode		cd_FOR_MODE or cd_NOR_MODE
	@param t_sb		time standby in cd_NOR_MODE
	@param filter	filter value: cd_FIL_OFF .. cd_FIL_x128
	@param osrs_t	oversampling value temperature: cd_OS_OFF..cd_OS_x16
	@param osrs_p	oversampling value pressure: cd_OS_OFF..cd_OS_x16
	@param osrs_h	oversampling value humidity: cd_OS_OFF..cd_OS_x16
	@returns void	*/
void cl_BME280::begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h) {	// init bme280
	cl_BME280::clf_readCalibData();
	cl_BME280::writeReg(0xF2, osrs_h);		//	write settings to config control registers 0xF2, 0xF4, 0xF5
	cl_BME280::writeReg(0xF4, ((osrs_t<<5) | (osrs_p<<2) | mode) );
	cl_BME280::writeReg(0xF5, ((t_sb << 5) | (filter << 2) | 0) );
};    

/*	@brief Read raw data (adc_ P T H) & calc it to compensate value
	@returns compensate value of T P H in structure var		*/
tph_stru cl_BME280::readTPH(void) {
	tph_stru lv_tph = { 0, 0, 0 };
	int32_t  adc_T, adc_H, adc_P, var1, var2, var3, var4, var5, t_fine;

	uint8_t lv_nregs = 8;
	uint8_t lv_regs[lv_nregs];		//	local temp array for store registers
	Wire.beginTransmission(clv_i2cAddr);   // addr of first byte raw data (adc_ P T H)
	Wire.write(0xF7);
	if (Wire.endTransmission() != 0) return lv_tph;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs)
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();
	adc_T = (((int32_t)lv_regs[3] << 16) | ((int32_t)lv_regs[4] << 8) | lv_regs[5]) >> 4;
	adc_P = (((int32_t)lv_regs[0] << 16) | ((int32_t)lv_regs[1] << 8) | lv_regs[2]) >> 4;
	adc_H = ((int32_t)lv_regs[6] << 8) | lv_regs[7];
#ifdef enDEBUG
	printf("adc_ T P H = %d %d %d \n", adc_T, adc_P, adc_H);
#endif

	//	Calc T
	if (adc_T == 0x800000) lv_tph.temp1 = 0;	// if the temperature module has been disabled return '0'
	else {
		var1 = (int32_t)((adc_T / 8) - ((int32_t)clv_cd.T1 * 2));
		var1 = (var1 * ((int32_t)clv_cd.T2)) / 2048;
		var2 = (int32_t)((adc_T / 16) - ((int32_t)clv_cd.T1));
		var2 = (((var2 * var2) / 4096) * ((int32_t)clv_cd.T3)) / 16384;
		t_fine = var1 + var2;
		lv_tph.temp1 = (float)((t_fine * 5 + 128) / 256) / 100.0;
	}

	//	Calc P
	if (adc_P == 0x800000) lv_tph.pres1 = 0;	// If the pressure module has been disabled return '0'
	else {
		int64_t var1_i64, var2_i64, var3_i64, var4_i64;

		var1_i64 = ((int64_t)t_fine) - 128000;
		var2_i64 = var1_i64 * var1_i64 * (int64_t)clv_cd.P6;
		var2_i64 = var2_i64 + ((var1_i64 * (int64_t)clv_cd.P5) * 131072);
		var2_i64 = var2_i64 + (((int64_t)clv_cd.P4) * 34359738368);
		var1_i64 = ((var1_i64 * var1_i64 * (int64_t)clv_cd.P3) / 256) +
			((var1_i64 * ((int64_t)clv_cd.P2) * 4096));
		// var3_i64 = ((int64_t)1) * 140737488355328;
		var3_i64 = 140737488355328;
		var1_i64 = (var3_i64 + var1_i64) * ((int64_t)clv_cd.P1) / 8589934592;

		if (var1_i64 == 0) lv_tph.pres1 = 0;	// avoid exception caused by division by zero
		else {
			var4_i64 = 1048576 - adc_P;
			var4_i64 = (((var4_i64 * 2147483648) - var2_i64) * 3125) / var1_i64;
			var1_i64 = (((int64_t)clv_cd.P9) * (var4_i64 / 8192) * (var4_i64 / 8192)) /	33554432;
			var2_i64 = (((int64_t)clv_cd.P8) * var4_i64) / 524288;
			var4_i64 = ((var4_i64 + var1_i64 + var2_i64) / 256) + (((int64_t)clv_cd.P7) * 16);

			lv_tph.pres1 = (float)var4_i64 / 256.0;
		}
	}

	//	Calc H
	if (adc_H == 0x8000) lv_tph.humi1 = 0;	// If the humidity module has been disabled return '0'
	else {
		var1 = t_fine - ((int32_t)76800);
		var2 = (int32_t)(adc_H * 16384);
		var3 = (int32_t)(((int32_t)clv_cd.H4) * 1048576);
		var4 = ((int32_t)clv_cd.H5) * var1;
		var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
		var2 = (var1 * ((int32_t)clv_cd.H6)) / 1024;
		var3 = (var1 * ((int32_t)clv_cd.H3)) / 2048;
		var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
		var2 = ((var4 * ((int32_t)clv_cd.H2)) + 8192) / 16384;
		var3 = var5 * var2;
		var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
		var5 = var3 - ((var4 * ((int32_t)clv_cd.H1)) / 16);
		var5 = (var5 < 0 ? 0 : var5);
		var5 = (var5 > 419430400 ? 419430400 : var5);
		lv_tph.humi1 = (float)(var5 / 4096) / 1024.0;
	}

	return lv_tph;
}



//============================================
//	BME680, Bosch Document rev.: 1.9, Date: February 2024, Document N: BST-BME680-DS001-09
//	cl_BME680, private metods (funcs)
//============================================
/*	@brief Read Calibration Data to structure variable clv_cd */
void cl_BME680::clf_readCalibData(void) {
	uint8_t lv_nregs = 23;
	uint8_t lv_regs[lv_nregs];		// temporary array for reading registers
	Wire.beginTransmission(clv_i2cAddr);	// first part request
	Wire.write(0x8A);						// Address of start calib. data (coeff.)
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {    // reading 23 regs from addr 0x8A
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();
// T1 0xE9/0xEA, T2 0x8A/0x8B, T3 0x8C
// P1 0x8E/0x8F, P2	0x90/0x91, P3 0x92, P4 0x94/0x95, P5 0x96/0x97, P6 0x99, P7 0x98, P8 0x9C/0x9D, P9	0x9E/0x9F, P10	0xA0
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
	Wire.beginTransmission(clv_i2cAddr);	// second part request 14 regs
	Wire.write(0xE1);						// Address of 2d part calibr data
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs) {   // reading
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();
// H1 0xE2<3:0>/0xE3, H2 0xE2<7:4>/0xE1, H3 0xE4, H4 0xE5, H5 0xE6, H6 0xE7, H7 0xE8
		clv_cd.H2 = ((uint16_t)lv_regs[0] << 4) | (lv_regs[1] >> 4);
		clv_cd.H1 = ((uint16_t)lv_regs[2] << 4) | (lv_regs[1] & 0x0F);
		clv_cd.H3 = lv_regs[3];
		clv_cd.H4 = lv_regs[4];
		clv_cd.H5 = lv_regs[5];
		clv_cd.H6 = lv_regs[6];
		clv_cd.H7 = lv_regs[7];
		clv_cd.T1 = lv_regs[9] << 8 | lv_regs[8];
// G1 0xED, G2 0xEB/0xEC, G3 0xEE, res_heat_range 0x02 <5:4>, res_heat_val 0x00
		clv_cd.G2 = lv_regs[11] << 8 | lv_regs[10];
		clv_cd.G1 = lv_regs[12];
		clv_cd.G3 = lv_regs[13];
	};
#ifdef enDEBUG
	printf("\nCalibrated data BME680:\n", clv_cd.T1, clv_cd.T2, clv_cd.T3);
	printf("T1-T3  = %d %d %d \n", clv_cd.T1, clv_cd.T2, clv_cd.T3);
    printf("P1-p10 = %d %d %d %d %d %d %d %d %d %d \n", clv_cd.P1, clv_cd.P2, clv_cd.P3, clv_cd.P4, clv_cd.P5, clv_cd.P6, clv_cd.P7, clv_cd.P8, clv_cd.P9, clv_cd.P10);
    printf("H1-H7  = %d %d %d %d %d %d %d \n", clv_cd.H1, clv_cd.H2, clv_cd.H3, clv_cd.H4, clv_cd.H5, clv_cd.H6, clv_cd.H7);
    printf("G1-G3  = %d %d %d \n\n", clv_cd.G1, clv_cd.G2, clv_cd.G3);
#endif
}

//============================================
//  cl_BME680, public metods (funcs)
//============================================
/*	@brief Send sensor command to Start Measuring 	*/
void cl_BME680::do1Meas(void) {    // mode FORCED_MODE DO 1 Measuring
	cl_BME680::writeReg(0x74, cl_BME680::readReg(0x74) | 0x01);
}

/*	@brief Test if sensor is Measuring 
	@return TRUE while bme680 is Measuring of FALSE when it is sleep	*/
bool cl_BME680::isMeas(void) {
	// Status reg 0x1D, check the bit <6> gas measuring = 1 and the bit <5> data measuring = 1
	return (bool)((cl_BME680::readReg(0x1D) & 0x60));
}

/*	@brief Read calibration data and Init sensor with default
	filter value: cd_FIL_x2 and oversampling value T P H : cd_OS_x16	*/
void cl_BME680::begin() {
	cl_BME680::begin(cd_FIL_x2, cd_OS_x16, cd_OS_x16, cd_OS_x16); // default: filter x2, oversampling TPH x16
}

/*	@brief Read calibration data and Init sensor with
	@param filter	filter value: cd_FIL_OFF .. cd_FIL_x128
	@param osrs_t	oversampling value temperature: cd_OS_OFF..cd_OS_x16
	@param osrs_p	oversampling value pressure: cd_OS_OFF..cd_OS_x16
	@param osrs_h	oversampling value humidity: cd_OS_OFF..cd_OS_x16
	@returns void	*/
void cl_BME680::begin(uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h) {
	cl_BME680::clf_readCalibData();	// Read calibration coefficients (data) to clas private (local) variable clv_cd
/*	Select mode, oversampling and filtering = Step 1, 2, 3. (3.2.2 Sensor configuration flow, p.16)
osrs_h bit <2:0> regs 0x72, osrs_t bit <7:5> regs 0x74, osrs_p bit <4:2> regs 0x72, mode bit <1:0>
Filtering value (cd_FIL_x..) to Config register address 0x75 bits <4:2>		*/
	cl_BME680::writeReg(0x72, osrs_h);
	cl_BME680::writeReg(0x74, ((osrs_t<<5) | (osrs_p<<2) | 0) );
	cl_BME680::writeReg(0x75, filter << 2);
};    

/*	@brief Set heating point 0..9 with
	@param lp_setPoint	number of setpoint 0..9
	@param lp_tagTemp	target temperature of heating, C 
	@param lp_duration	time of heating, msec
	@param lp_ambTemp	ambient temperature of sensor, C	*/
void cl_BME680::initGasPointX(uint8_t lp_setPoint, uint16_t lp_tagTemp, uint16_t lp_duration, int16_t lp_ambTemp) {
	//  Up to 10 different hot plate temperature set points can be configured 
	//	by setting the registers res_heat_X and gas_wait_X, where X = 0…9.
	//	The internal heater control loop operates on the resistance of the heater structure.
	//  Hence, the user first needs to convert the target temperature 
	//	into a device specific target resistance (res_heat_X))
	//  before writing the resulting register code into the sensor memory map.

	//	Res_heat_X	5Ah-63h,
	//  Gas_wait_X	64h-6Dh.
	//  Idac_heat_X	50h-59h will calc by BME680 itself

	//	Step 4 - Enable GAS conversion. run_gas =1 (set bit <4> address reg 0x71) and 
	//	Step 5 - Select index of heater set-point 0-9. nb_conv = 0 (bits <3:0> address reg 0x71)
	if (lp_setPoint > 9) lp_setPoint = 9;
	cl_BME680::writeReg(0x71, (0x10 | lp_setPoint) );

	//	Step 6 - Define heater duration Temp in ms, reg gas_wait_X, where X = 0…9. 
	//	registers address 0x64-0x6D, bit <5:0> ms and bit <7:6> is multiplier
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

	//	Set heater Temp = Step 7.
	//	Convert temperature to register cspecific val. Set res_heat_X (reg 0x5A-0x63)
	// G1, G2, G3	- calibration parameters,
	// targTemp 	- the target heater temperature in degree Celsius,
	// ambTemp	 	- the ambient temperature (hardcoded or read from temperature sensor),
	// var5 		- the target heater resistance in Ohm,
	// res_heat_X	- the device specific target resistance value that needs to be stored in register,
	//	where 'X' corresponds to the temperature profilenumber between 0 and 9 stored in regs 0x5A-0x63
	// res_heat_range 	- the heater range stored in register address 0x02 <5:4>, and
	// res_heat_val - the heater resistance correction factor stored in register address 0x00, signed, value from -128 to 127
	int32_t var1 = (((int32_t)lp_ambTemp * clv_cd.G3) / 10) << 8;
	int32_t var2 = (clv_cd.G1 + 784) * (((((clv_cd.G2 + 154009) * lp_tagTemp * 5) / 100) + 3276800) / 10);
	int32_t var3 = var1 + (var2 >> 1);
	int8_t  res_heat_val   = cl_BME680::readReg(0x00);
	uint8_t res_heat_range = (cl_BME680::readReg(0x02) >> 4) & 0x03;
	int32_t var4 = (var3 / (res_heat_range + 4));
	int32_t var5 = (131 * res_heat_val) + 65536;
	int32_t res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
	uint8_t res_heat_X = (uint8_t)((res_heat_x100 + 50) / 100);
	cl_BME680::writeReg(0x5A + lp_setPoint, res_heat_X);
}

/*	@brief Read raw data (adc_ P T H G) & calc it to compensate value
	@returns structure T P H G	*/
tphg_stru cl_BME680::readTPHG(void) {
	tphg_stru lv_tphg = { 0, 0, 0, 0 };
	uint32_t  adc_T, adc_P, adc_H, adc_G;
	int32_t lv_var1, lv_var2, lv_var3, t_fine, temp_comp;

	// read raw data (adc_ P T H G) from addr 0x1F to 0x1B at once I2C request
	uint8_t lv_nregs = 13;
	uint8_t lv_regs[lv_nregs];		//	temp array
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(0x1F);
	if (Wire.endTransmission() != 0) return lv_tphg;
	if (Wire.requestFrom(clv_i2cAddr, lv_nregs) == lv_nregs)
		for (uint8_t i = 0; i < lv_nregs; i++) lv_regs[i] = Wire.read();
	adc_P = (uint32_t)0 | (lv_regs[0] << 12) | (lv_regs[1] << 4) | (lv_regs[2] >> 4);
	adc_T = (uint32_t)0 | (lv_regs[3] << 12) | (lv_regs[4] << 4) | (lv_regs[5] >> 4);
	adc_H = (uint32_t)0 | (lv_regs[6] << 8) | lv_regs[7];
	adc_G = (uint32_t)0 | ((uint32_t)lv_regs[11] << 2) | (uint32_t)(lv_regs[12] >> 6);
	uint8_t gas_range = lv_regs[12] & 0x0F;
	uint8_t range_switching_error = ( (int8_t)(cl_BME680::readReg(0x04) & 0xF0) / 16 );
#ifdef enDEBUG
	uint8_t lv_status = cl_BME680::readReg(0x1D);
	if (lv_status & 0b10000000) Serial.println("new_data_0 = 1, moment when new measuring data have been arrive.");
	if (lv_status & 0b01000000) Serial.println("gas_measuring = 1, moment gas data is measuring.");
	if (lv_status & 0b00100000) Serial.println("measuring = 1, moment raw data is measuring.");
	if (!(lv_regs[12] & 0b00100000)) Serial.println("Gas Not Valid bit<5> = 0 !!!");	// Test for Ok gas measuring
	if (!(lv_regs[12] & 0b00010000)) Serial.println("Heat Not Stable bit<4> = 0 !!!");	// Test for Ok gas preheating
	printf("adc_ T P H G =  %d %d %d %d \n", adc_T, adc_P, adc_H, adc_G);
#endif

	// Calc T, where par_t1, par_t2 and par_t3 are calibration parameters,
	// adc_T - the raw temperature data, t_fine - temperature that will use in future calc
	if (adc_T == 0x800000) lv_tphg.temp1 = 0;	// if the temperature module has been disabled return '0'
	else {
		lv_var1 = ((int32_t)adc_T >> 3) - ((int32_t)clv_cd.T1 << 1);
		lv_var2 = (lv_var1 * (int32_t)clv_cd.T2) >> 11;
		lv_var3 = ((((lv_var1 >> 1) * (lv_var1 >> 1)) >> 12) * ((int32_t)clv_cd.T3 << 4)) >> 14;
		t_fine = lv_var2 + lv_var3;
		lv_tphg.temp1 = ((float)(((t_fine * 5) + 128) >> 8)) / 100;
	}

	// Calc P, where par_p1, par_p2, …, par_p10 are calibration parameters,
	// adc_P - the raw pressure data, press_comp - the compensated pressure in Pascal.
	if (adc_P == 0x800000) {
		lv_tphg.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		uint32_t press_comp;
		lv_var1 = ((int32_t)t_fine >> 1) - 64000;
		lv_var2 = ((((lv_var1 >> 2) * (lv_var1 >> 2)) >> 11) * (int32_t)clv_cd.P6) >> 2;
		lv_var2 = lv_var2 + ((lv_var1 * (int32_t)clv_cd.P5) << 1);
		lv_var2 = (lv_var2 >> 2) + ((int32_t)clv_cd.P4 << 16);
		lv_var1 = (((((lv_var1 >> 2) * (lv_var1 >> 2)) >> 13) * ((int32_t)clv_cd.P3 << 5)) >> 3) + (((int32_t)clv_cd.P2 * lv_var1) >> 1);
		lv_var1 = lv_var1 >> 18;
		lv_var1 = ((32768 + lv_var1) * (int32_t)clv_cd.P1) >> 15;
		press_comp = 1048576 - adc_P;
		press_comp = (uint32_t)((press_comp - (lv_var2 >> 12)) * ((uint32_t)3125));
		if (press_comp >= (1 << 30))	//	1073741824
			press_comp = ((press_comp / (uint32_t)lv_var1) << 1);
		else
			press_comp = ((press_comp << 1) / (uint32_t)lv_var1);
		lv_var1 = ((int32_t)clv_cd.P9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
		lv_var2 = ((int32_t)(press_comp >> 2) * (int32_t)clv_cd.P8) >> 13;
		lv_var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)clv_cd.P10) >> 17;
		press_comp = (int32_t)(press_comp)+((lv_var1 + lv_var2 + lv_var3 + ((int32_t)clv_cd.P7 << 7)) >> 4);
		lv_tphg.pres1 = (float)press_comp;
	}

	// Calc H, where par_h1, par_h2, …, par_h7 are calibration parameters,
	// hum_adc is the raw humidity data, hum_comp - the compensated humidity in percent.
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

	// Calc of GAS sensor resistance consists of 4 steps:
	// 1. Read raw gas sensor resistance data (i.e. ADC value) adc_G	
	// 		register address 0x2A<9:2>/0x2B<7:6> 	=> adc_G
	// 2. Read range switching error - a calibration parameter,
	// 		register address 0x04 bits <7:4>		=> range_switching_error
	// 3. Read gas ADC range (gas_range) of the measured gas sensor resistance, (see Section 5.3.4)
	// 		register address 0x2B bits <3:0>	   	=> gas_range
	// 4. Convert ADC value (adc_G) into compensated gas sensor resistance (gas_res) in Ohm (kOm)
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

		int64_t		var1, var3;
		uint64_t	var2;
		var1 = (int64_t)((1340 + (5 * (int64_t)range_switching_error)) * ((int64_t)uintTab1[gas_range])) >> 16;
		var2 = (((int64_t)((int64_t)adc_G << 15) - (int64_t)(16777216)) + var1);
		var3 = (((int64_t)uintTab2[gas_range] * (int64_t)var1) >> 9);
		uint32_t gas_res = (uint32_t)((var3 + ((int64_t)var2 >> 1)) / (int64_t)var2);
		lv_tphg.gasr1 = ((float)gas_res) / 1000;	//	resistance kOm
	}

	return lv_tphg;
}
//============================================================================================================
