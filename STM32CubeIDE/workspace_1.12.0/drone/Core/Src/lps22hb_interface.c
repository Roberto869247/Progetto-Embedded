/*
 * Interfaccia per l'acquisizione dati dal sensore LPS22HB
 */

#include <global_variables.h>
#include "lps22hb_interface.h"
#include "bin_semaphores.h"
float pressure;
stmdev_ctx_t lps22hb_dev_ctx;

int32_t lps22hb_platform_write(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	if (handle == &hi2c1) {
		/* Write multiple command */
		HAL_I2C_Mem_Write(handle, LPS22HB_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT,
				bufp, len, 1000);
	}
	return 0;
}

int32_t lps22hb_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	if (handle == &hi2c1) {
		/* Read multiple command */
		HAL_I2C_Mem_Read(handle, LPS22HB_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp,
				len, 1000);
	}
	return 0;
}

void LPS22HB_init() {
	uint8_t id, rst = 0;

	lps22hb_dev_ctx.write_reg = lps22hb_platform_write;
	lps22hb_dev_ctx.read_reg = lps22hb_platform_read;
	lps22hb_dev_ctx.handle = &hi2c1;

	// Poll device
	while (id != LPS22HB_ID) {
		lps22hb_device_id_get(&lps22hb_dev_ctx, &id);
	}

	// Restore default configuration
	lps22hb_reset_set(&lps22hb_dev_ctx, PROPERTY_ENABLE);
	do {
		lps22hb_reset_get(&lps22hb_dev_ctx, &rst);
	} while (rst);

	// Enable Block Data Update
	//lps22hb_block_data_update_set(&lps22hb_dev_ctx, PROPERTY_ENABLE);

	// Enable low pass filter on output
	lps22hb_low_pass_filter_mode_set(&lps22hb_dev_ctx, LPS22HB_LPF_ODR_DIV_2);

	// Set Data-ready signal on INT_DRDY pin
	//lps22hb_drdy_on_int_set(&lps22hb_dev_ctx, PROPERTY_ENABLE);

	// Set Output Data Rate
	lps22hb_data_rate_set(&lps22hb_dev_ctx, LPS22HB_ODR_10_Hz);
}

void LPS22HB_dataRead() {
	uint8_t reg;
	uint32_t temp; //variabile temporanea

	/* Read output only if new value is available */
	lps22hb_press_data_ready_get(&lps22hb_dev_ctx, &reg);
	if (reg) {
		//memset(lps22hb_data.raw.u8bit, 0x00, sizeof(int32_t));
		lps22hb_pressure_raw_get(&lps22hb_dev_ctx, &temp);
		if(osSemaphoreAcquire(binSemaphorePresHandle, 0)==osOK){
			pressure = lps22hb_from_lsb_to_hpa(temp);
			osSemaphoreRelease(binSemaphorePresHandle);
		}
	}
}
