/************************************************************************************
Library for test and use Bosch i2c sensor bme280, bmp280
by Igor Mkprog, mkprogigor@gmail.com

use examples:
https://github.com/AlexGyver/GyverLibs/blob/master/GyverBME280/
https://github.com/farmerkeith/BMP280-library/
https://github.com/farmerkeith/I2CScanner/

V1.2 from 19.06.2025
************************************************************************************/

#include <mkigor_BMx280.h>

uint8_t _reg_0xF2 = 0x07;	//	config regs osr_h[2:0]
uint8_t _reg_0xF4 = 0xFE;	//	config regs osr_t[7:5] osr_p[4:2] mode[1:0]
uint8_t _reg_0xF5 = 0x9C;	//	config regs t_sb[7:5] filter[4:2] spi3w_en[0]

uint8_t cl_BMP280::check(uint8_t _lv_i2caddr=clv_i2cAddr) {
	// check is it conected bm(e,p)280, if YES ret chip_code, if NO ret 0
	clv_i2cAddr = _lv_i2caddr;
	Wire.begin();
	uint8_t lv_error;
    Wire.beginTransmission(clv_i2cAddr);
    lv_error = Wire.endTransmission();
    if (lv_error == 0) {
        Wire.beginTransmission(clv_i2cAddr);
        Wire.write(0xD0);   // Select register 0xD0 hex address of chip_id
        Wire.endTransmission();
        Wire.requestFrom(clv_i2cAddr, 1);
		if (Wire.available() == 1) {
			clv_codeChip = Wire.read();
			reset();
			return clv_codeChip;
		}
    }
    return 0;
	/*  fn return 0x58=BMP280, 0x60=BME280, 0x61=BME680.
	i2c address 0x76, 0x77 possible for BMP280 or BME280 or BME680 or MS5607,MS5611,MS5637
	note: address 0x77 may be BMP085, BMP180. CHECK IT */
}

void cl_BMP280::begin() {	// defaults are 16x; Normal mode; 0.5ms, no filter, I2C
	begin(cd_FOR_MODE, cd_SB_500MS, cd_FIL_x16, cd_OS_x16, cd_OS_x16); // Forse mode, sleep 500ms, filter x16, t p h x16
  }

void cl_BMP280::begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p) {	// init bme280
	cl_BMP280::read_calibCoef();
	_reg_0xF4 = (osrs_t<<5) | (osrs_p<<2) | mode;
	_reg_0xF5 = (t_sb << 5) | (filter << 2) | 0x00;
	
	cl_BMP280::writeReg(0xF4, _reg_0xF4);
	cl_BMP280::writeReg(0xF5, _reg_0xF5);
};    

void cl_BME280::begin() {	// defaults are 16x; Normal mode; 0.5ms, no filter, I2C
	begin(cd_FOR_MODE, cd_SB_500MS, cd_FIL_x16, cd_OS_x16, cd_OS_x16, cd_OS_x16); // Forse mode, sleep 500ms, filter x16, t p h x16
}

void cl_BME280::begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h) {	// init bme280
	cl_BME280::read_calibCoef();
	_reg_0xF2 = osrs_h;
	_reg_0xF4 = (osrs_t<<5) | (osrs_p<<2) | mode;
	_reg_0xF5 = (t_sb << 5) | (filter << 2) | 0x00;
	
	cl_BME280::writeReg(0xF2, _reg_0xF2);
	cl_BME280::writeReg(0xF4, _reg_0xF4);
	cl_BME280::writeReg(0xF5, _reg_0xF5);
};    




bool BMx280::is_meas(void) {	// returns TRUE while bme280 is measuring								
	return (bool)((BMx280::_f_read_reg(0xF3) & 0x08) >> 3);  	 // read status register & mask bit "measuring"
}

void BMx280::do1meas(void) {    // mode FORCED_MODE do 1 measuring
	BMx280::_f_write_reg(0xF4, ((_reg_0xF4 & 0xFC) | 0x02));
}

struct_tph BMx280::read_TPH(void) {
    #define BME280_S32_t int32_t		// make compatible code from BOSCH
    #define BME280_U32_t uint32_t
    #define BME280_S64_t int64_t
    struct_tph lv_tph = {0, 0, 0};
	int32_t  adc_T;
	uint32_t adc_P;
	int32_t  adc_H;
	uint8_t _lv_regs[8];

	Wire.beginTransmission(gv_i2c_address);   // addr of first byte raw data Humi
	Wire.write(0xF7);
	if (Wire.endTransmission() != 0) return lv_tph;

	if (gv_code_chip != BMP280) {	// reading 8 regs
		if (Wire.requestFrom(gv_i2c_address, 8) == 8) for (uint8_t i = 0; i < 8; i++) _lv_regs[i] = Wire.read();
		adc_T = ((_lv_regs[3] << 16) | (_lv_regs[4] << 8) | _lv_regs[5]) >> 4;
		adc_P = ((_lv_regs[0] << 16) | (_lv_regs[1] << 8) | _lv_regs[2]) >> 4;
		adc_H = (_lv_regs[6] << 8) | _lv_regs[7];
	}
	else {	// reading 6 regs
		if (Wire.requestFrom(gv_i2c_address, 6) == 6) for (uint8_t i = 0; i < 6; i++) _lv_regs[i] = Wire.read();
		adc_T = ((_lv_regs[3] << 16) | (_lv_regs[4] << 8) | _lv_regs[5]) >> 4;
		adc_P = ((_lv_regs[0] << 16) | (_lv_regs[1] << 8) | _lv_regs[2]) >> 4;
	}

	// Serial.print("adc_T = ");	Serial.println(adc_T);
	// Serial.print("adc_P = ");	Serial.println(adc_P);
	// Serial.print("gv_i2c_address = ");	Serial.println(gv_i2c_address, HEX);
	// Serial.print(_cd.dig_T1);	Serial.print(_cd.dig_T2);	Serial.print(_cd.dig_T3);
	// Serial.print(_cd.dig_P1);	Serial.print(_cd.dig_P2);	Serial.print(_cd.dig_P3);
	// Serial.print(_cd.dig_P4);	Serial.print(_cd.dig_P5);	Serial.print(_cd.dig_P6);
	// Serial.print(_cd.dig_P7);	Serial.print(_cd.dig_P8);	Serial.println(_cd.dig_P9);

	// t_fine carries fine temperature as global value
	BME280_S32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tph.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		// adc_T >>= 4;
		lv_var1 = ((((adc_T >> 3) - ((BME280_S32_t)_cd.dig_T1 << 1))) * ((BME280_S32_t)_cd.dig_T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((BME280_S32_t)_cd.dig_T1)) * ((adc_T >> 4) - ((BME280_S32_t)_cd.dig_T1))) >> 12) * ((BME280_S32_t)_cd.dig_T3)) >> 14;
		t_fine = lv_var1 + lv_var2;
		lv_tph.temp1  = ((float)((t_fine * 5 + 128) >> 8)) / 100;
	}

	BME280_S64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tph.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		// adc_P >>= 4;	// Start pressure converting
		var1 = ((BME280_S64_t)t_fine) - 128000;
		var2 = var1 * var1 * (BME280_S64_t)_cd.dig_P6;
		var2 = var2 + ((var1 * (BME280_S64_t)_cd.dig_P5) << 17);
		var2 = var2 + (((BME280_S64_t)_cd.dig_P4) << 35);
		var1 = ((var1 * var1 * (BME280_S64_t)_cd.dig_P3) >> 8) + ((var1 * (BME280_S64_t)_cd.dig_P2) << 12);
		var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)_cd.dig_P1) >> 33;
		if (var1 == 0) {
			lv_tph.pres1 = 0;     // avoid exception caused by division by zero
		}
		else {
			p = 1048576 - adc_P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (((BME280_S64_t)_cd.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
			var2 = (((BME280_S64_t)_cd.dig_P8) * p) >> 19;
			p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)_cd.dig_P7) << 4);
			lv_tph.pres1 =((float)((BME280_U32_t)((p * 100) / 256))) / 100;
		}
	}

	if (gv_code_chip != BMP280) {
		BME280_S32_t v_x1_u32r;
		if (adc_H == 0x8000) {
			lv_tph.humi1 = 0;	// If the humidity module has been disabled return '0'
		}
		else {
			v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
			v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)_cd.dig_H4) << 20) - (((BME280_S32_t)_cd.dig_H5) *
				v_x1_u32r)) + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *
					((BME280_S32_t)_cd.dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)_cd.dig_H3)) >> 11) +
						((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)_cd.dig_H2) + 8192) >> 14));
			v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)_cd.dig_H1)) >> 4));
			v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
			v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
			lv_tph.humi1 = ((float)((BME280_U32_t)(((v_x1_u32r >> 12) * 1000) / 1024))) / 1000;
		}
	}
	return lv_tph;
}

BMx280::BMx280() {};	// create an object of class bme280

//============================================
//    private metods (funcs)
//============================================

uint8_t BMx280::_f_read_reg(uint8_t address) {    // read 1 byte bme280 register
	Wire.beginTransmission(gv_i2c_address);
	Wire.write(address);
	if (Wire.endTransmission() != 0) return 0;
	if (Wire.requestFrom(gv_i2c_address, 1) == 1) return Wire.read();
	else return 0;
}

bool BMx280::_f_write_reg(uint8_t address, uint8_t data) {   //  write 1 byte to bme280 register
	Wire.beginTransmission(gv_i2c_address);
	Wire.write(address);
	Wire.write(data);
	if (Wire.endTransmission() == 0) return true;
	else return false;
}

bool BMx280::_f_reset(void) {     //  software reset of bme280
	if (BMx280::_f_write_reg(0x0E, 0xB6)) return true;
    else return false;
}

void BMx280::_f_read_calibr_coeff(void) {
	uint8_t _tv_regs[26];

	Wire.beginTransmission(gv_i2c_address);   // first part request
	Wire.write(0x88);
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(gv_i2c_address, 26) == 26) {    // reading 26 regs
		for (uint8_t i = 0; i < 25; i++) _tv_regs[i] = Wire.read();

		_cd.dig_T1 = _tv_regs[1] << 8 | _tv_regs[0];   // form struct
		_cd.dig_T2 = _tv_regs[3] << 8 | _tv_regs[2];
		_cd.dig_T3 = _tv_regs[5] << 8 | _tv_regs[4];
		_cd.dig_P1 = _tv_regs[7] << 8 | _tv_regs[6];
		_cd.dig_P2 = _tv_regs[9] << 8 | _tv_regs[8];
		_cd.dig_P3 = _tv_regs[11] << 8 | _tv_regs[10];
		_cd.dig_P4 = _tv_regs[13] << 8 | _tv_regs[12];
		_cd.dig_P5 = _tv_regs[15] << 8 | _tv_regs[14];
		_cd.dig_P6 = _tv_regs[17] << 8 | _tv_regs[16];
		_cd.dig_P7 = _tv_regs[19] << 8 | _tv_regs[18];
		_cd.dig_P8 = _tv_regs[21] << 8 | _tv_regs[20];
		_cd.dig_P9 = _tv_regs[23] << 8 | _tv_regs[22];
		_cd.dig_H1 = _tv_regs[25];
	}

	if (gv_code_chip != BMP280) {

		Wire.beginTransmission(gv_i2c_address);   // second part request
		Wire.write(0xE1);
		Wire.endTransmission();
		if (Wire.requestFrom(gv_i2c_address, 7) == 7) {   // reading
			for (uint8_t i = 0; i < 7; i++) _tv_regs[i] = Wire.read();
			_cd.dig_H2 = _tv_regs[1] << 8 | _tv_regs[0]; // (Wire.read() | (Wire.read() << 8));
			_cd.dig_H3 = _tv_regs[2];
			_cd.dig_H4 = ((int16_t)_tv_regs[3] << 4) | (_tv_regs[4] & 0x0F);
			_cd.dig_H5 = ((int16_t)_tv_regs[5] << 4) | ((_tv_regs[4] & 0xF0) >> 4);
			_cd.dig_H6 = _tv_regs[6];
		};

	}
}