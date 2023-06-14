/*
 * Interfaccia per l'acquisizione dati dal sensore LSM303AGR
 */

#include "lsm303agr_interface.h"
#include "bin_semaphores.h"
#include <global_variables.h>

stmdev_ctx_t lsm303agr_dev_ctx_xl;
stmdev_ctx_t lsm303agr_dev_ctx_mg;
lsm303agr_data magnetometer;
lsm303agr_data accellerometer1;

int32_t lsm303agr_platform_write(void *handle, uint8_t Reg,
		uint8_t *Bufp, uint16_t len) {
	uint32_t i2c_add = (uint32_t) handle;
	if (i2c_add == LSM303AGR_I2C_ADD_XL) {
		/* enable auto incremented in multiple read/write commands */
		Reg |= 0x80;
	}
	HAL_I2C_Mem_Write(&hi2c1, i2c_add, Reg, I2C_MEMADD_SIZE_8BIT, Bufp, len,
			1000);
	return 0;
}

int32_t lsm303agr_platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
		uint16_t len) {
	uint32_t i2c_add = (uint32_t) handle;
	if (i2c_add == LSM303AGR_I2C_ADD_XL) {
		/* enable auto incremented in multiple read/write commands */
		Reg |= 0x80;
	}
	HAL_I2C_Mem_Read(&hi2c1, (uint8_t) i2c_add, Reg, I2C_MEMADD_SIZE_8BIT, Bufp,
			len, 1000);
	return 0;
}

void LSM303AGR_init() {
	uint8_t id, rst = 0;

	// Accelerometro
	lsm303agr_dev_ctx_xl.write_reg = lsm303agr_platform_write;
	lsm303agr_dev_ctx_xl.read_reg = lsm303agr_platform_read;
	lsm303agr_dev_ctx_xl.handle = (void*) LSM303AGR_I2C_ADD_XL;

	// Poll device
	while (id != LSM303AGR_ID_XL) {
		lsm303agr_xl_device_id_get(&lsm303agr_dev_ctx_xl, &id);
	}

	/* Enable Block Data Update */
	lsm303agr_xl_block_data_update_set(&lsm303agr_dev_ctx_xl, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lsm303agr_xl_data_rate_set(&lsm303agr_dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);

	/* Set accelerometer full scale */
	lsm303agr_xl_full_scale_set(&lsm303agr_dev_ctx_xl, LSM303AGR_2g);

	/* Enable temperature sensor */
	lsm303agr_temperature_meas_set(&lsm303agr_dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
	/* Set device in continuos mode */
	lsm303agr_xl_operating_mode_set(&lsm303agr_dev_ctx_xl, LSM303AGR_HR_12bit);

	// Magnetometro
	id = 0;
	rst = 0;

	lsm303agr_dev_ctx_mg.write_reg = lsm303agr_platform_write;
	lsm303agr_dev_ctx_mg.read_reg = lsm303agr_platform_read;
	lsm303agr_dev_ctx_mg.handle = (void*) LSM303AGR_I2C_ADD_MG;

	// Poll device
	while (id != LSM303AGR_ID_MG) {
		lsm303agr_mag_device_id_get(&lsm303agr_dev_ctx_mg, &id);
	}

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(&lsm303agr_dev_ctx_mg, PROPERTY_ENABLE);
	do {
		lsm303agr_mag_reset_get(&lsm303agr_dev_ctx_mg, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm303agr_mag_block_data_update_set(&lsm303agr_dev_ctx_mg, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lsm303agr_mag_data_rate_set(&lsm303agr_dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);

	/* Set / Reset magnetic sensor mode */
	lsm303agr_mag_set_rst_mode_set(&lsm303agr_dev_ctx_mg,
			LSM303AGR_SENS_OFF_CANC_EVERY_ODR);

	/* Enable temperature compensation on mag sensor */
	lsm303agr_mag_offset_temp_comp_set(&lsm303agr_dev_ctx_mg, PROPERTY_ENABLE);

	/* Set magnetometer in continuos mode */
	lsm303agr_mag_operating_mode_set(&lsm303agr_dev_ctx_mg,
			LSM303AGR_CONTINUOUS_MODE);
}

void LSM303AGR_dataReadAcc() {
	/* Read output only if new value is available */
	lsm303agr_reg_t reg;
	lsm303agr_xl_status_get(&lsm303agr_dev_ctx_xl, &reg.status_reg_a);
	int16_t temp[3];
	float toMS2 = 9.81;

	if (reg.status_reg_a.zyxda) {
		/* Read accelerometer data */
		//memset(lsm303agr_xl_data.raw.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm303agr_acceleration_raw_get(&lsm303agr_dev_ctx_xl, temp);
		if(osSemaphoreAcquire(binSemaphoreAcc1Handle, 0)==osOK){
			accellerometer1.x = lsm303agr_from_fs_2g_hr_to_mg(temp[0])/toMS2;
			accellerometer1.y = lsm303agr_from_fs_2g_hr_to_mg(temp[1])/toMS2;
			accellerometer1.z = lsm303agr_from_fs_2g_hr_to_mg(temp[2])/toMS2;
			osSemaphoreRelease(binSemaphoreAcc1Handle);
		}
	}
}

void LSM303AGR_dataReadMag() {
	/* Read output only if new value is available */
	lsm303agr_reg_t reg;
	int16_t temp[3]; //variabile temporanea
	lsm303agr_mag_status_get(&lsm303agr_dev_ctx_mg, &reg.status_reg_m);
	if (reg.status_reg_m.zyxda) {
		/* Read magnetic field data */
		//memset(lsm303agr_mg_data.raw.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm303agr_magnetic_raw_get(&lsm303agr_dev_ctx_mg, temp);
		if(osSemaphoreAcquire(binSemaphoreMagHandle, 0)==osOK){
			magnetometer.x = lsm303agr_from_lsb_to_mgauss(temp[0]);
			magnetometer.y = lsm303agr_from_lsb_to_mgauss(temp[1]);
			magnetometer.z = lsm303agr_from_lsb_to_mgauss(temp[2]);
			osSemaphoreRelease(binSemaphoreMagHandle);
		}
	}
}

