/*
 * Interfaccia per l'acquisizione dati dal sensore LSM303AGR
 */

#ifndef INC_LSM303AGR_INTERFACE_H_
#define INC_LSM303AGR_INTERFACE_H_

#include <string.h>
#include "lsm303agr_reg.h"
#include "i2c.h"

extern stmdev_ctx_t lsm303agr_dev_ctx_xl;  // Registri per il controllo periferica
extern stmdev_ctx_t lsm303agr_dev_ctx_mg;  // Registri per il controllo periferica

int32_t lsm303agr_platform_write(void *handle, uint8_t reg,uint8_t *bufp, uint16_t len);
int32_t lsm303agr_platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);

void LSM303AGR_init();
void LSM303AGR_dataReadAcc();
void LSM303AGR_dataReadMag();

#endif /* INC_U_LSM303AGR_DRIVER_H_ */
