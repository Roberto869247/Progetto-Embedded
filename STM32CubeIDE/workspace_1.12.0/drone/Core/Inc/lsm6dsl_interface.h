/*
 * Interfaccia per l'acquisizione dati dal sensore LSM6DSL
 */

#ifndef INC_LSM6DSL_INTERFACE_H_
#define INC_LSM6DSL_INTERFACE_H_

#include <string.h>
#include "lsm6dsl_reg.h"
#include "i2c.h"

extern stmdev_ctx_t lsm6dsl_dev_ctx;     // Registri controllo periferica

int32_t lsm6dsl_platform_write(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);
int32_t lsm6dsl_platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);

void LSM6DSL_init();
void LSM6DSL_dataReadAcc();
void LSM6DSL_dataReadGyro();

#endif /* U_LSM6DSL_DRIVER_H_ */
