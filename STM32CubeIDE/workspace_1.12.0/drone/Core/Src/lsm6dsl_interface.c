/*
 * Interfaccia per l'acquisizione dati dal sensore LSM6DSL
 */

#include "lsm6dsl_interface.h"
#include "gyro.h"
#include "bin_semaphores.h"
#include <global_variables.h>

lsm6dsl_data gyroscope;
lsm6dsl_data accellerometer2;
stmdev_ctx_t lsm6dsl_dev_ctx;

int32_t lsm6dsl_platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
		uint16_t len) {
	if (handle == &hi2c1) {
		HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_H, Reg,
		I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
	}
	return 0;
}

int32_t lsm6dsl_platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
		uint16_t len) {
	if (handle == &hi2c1) {
		HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_H, Reg,
		I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
	}
	return 0;
}

void LSM6DSL_init() {
	uint8_t id, rst = 0;

	lsm6dsl_dev_ctx.write_reg = lsm6dsl_platform_write;
	lsm6dsl_dev_ctx.read_reg = lsm6dsl_platform_read;
	lsm6dsl_dev_ctx.handle = &hi2c1;

	// Poll device
	while (id != LSM6DSL_ID) {
		lsm6dsl_device_id_get(&lsm6dsl_dev_ctx, &id);
	}

	/*
	 *  Restore default configuration
	 */
	lsm6dsl_reset_set(&lsm6dsl_dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dsl_reset_get(&lsm6dsl_dev_ctx, &rst);
	} while (rst);
	/*
	 *  Enable Block Data Update
	 */
	lsm6dsl_block_data_update_set(&lsm6dsl_dev_ctx, PROPERTY_ENABLE);
	/*
	 * Set Output Data Rate
	 */
	lsm6dsl_xl_data_rate_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_ODR_12Hz5);
	lsm6dsl_gy_data_rate_set(&lsm6dsl_dev_ctx, LSM6DSL_GY_ODR_12Hz5);
	/*
	 * Set full scale
	 */
	lsm6dsl_xl_full_scale_set(&lsm6dsl_dev_ctx, LSM6DSL_2g);
	lsm6dsl_gy_full_scale_set(&lsm6dsl_dev_ctx, LSM6DSL_2000dps);

	/*
	 * Configure filtering chain(No aux interface)
	 */
	/* Accelerometer - analog filter */
	lsm6dsl_xl_filter_analog_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);

	/* Accelerometer - LPF1 path ( LPF2 not used )*/
	//lsm6dsl_xl_lp1_bandwidth_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
	/* Accelerometer - LPF1 + LPF2 path */
	lsm6dsl_xl_lp2_bandwidth_set(&lsm6dsl_dev_ctx,
			LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);

	/* Accelerometer - High Pass / Slope path */
	//lsm6dsl_xl_reference_mode_set(&lsm6dsl_dev_ctx, PROPERTY_DISABLE);
	//lsm6dsl_xl_hp_bandwidth_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);
	/* Gyroscope - filtering chain */
	lsm6dsl_gy_band_pass_set(&lsm6dsl_dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);
}

void LSM6DSL_dataReadAcc() {
	/* Read output only if new value is available */
	lsm6dsl_reg_t reg;
	lsm6dsl_status_reg_get(&lsm6dsl_dev_ctx, &reg.status_reg);
	int16_t temp[3];
	float toMS2 = 9.81;
	if (reg.status_reg.xlda) {
		lsm6dsl_acceleration_raw_get(&lsm6dsl_dev_ctx, temp);
		if(osSemaphoreAcquire(binSemaphoreAcc2Handle, 0)==osOK){
			accellerometer2.x = lsm6dsl_from_fs2g_to_mg(temp[0])/toMS2;
			accellerometer2.y = lsm6dsl_from_fs2g_to_mg(temp[1])/toMS2;
			accellerometer2.z = lsm6dsl_from_fs2g_to_mg(temp[2])/toMS2;
			osSemaphoreRelease(binSemaphoreAcc2Handle);
		}
	}
}

void LSM6DSL_dataReadGyro() {
	/* Read output only if new value is available */
	lsm6dsl_reg_t reg;
	lsm6dsl_status_reg_get(&lsm6dsl_dev_ctx, &reg.status_reg);
	int16_t temp[3];  //variabile temporanea
	if (reg.status_reg.gda) {
		lsm6dsl_angular_rate_raw_get(&lsm6dsl_dev_ctx, temp);
		if(osSemaphoreAcquire(binSemaphoreGyrHandle, 0)==osOK){
			gyroscope.x = lsm6dsl_from_fs2000dps_to_mdps(temp[0]);
			gyroscope.y = lsm6dsl_from_fs2000dps_to_mdps(temp[1]);
			gyroscope.z = lsm6dsl_from_fs2000dps_to_mdps(temp[2]);
			osSemaphoreRelease(binSemaphoreGyrHandle);
		}
	}
}

