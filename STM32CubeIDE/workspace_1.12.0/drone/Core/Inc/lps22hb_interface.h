/*
 * Interfaccia per l'acquisizione dati dal sensore LPS22HB
 */

#ifndef INC_LPS22HB_INTERFACE_H_
#define INC_LPS22HB_INTERFACE_H_

#include <string.h>
#include "lps22hb_reg.h"
#include "i2c.h"

extern stmdev_ctx_t lps22hb_dev_ctx;  // Registro controllo periferica

int32_t lps22hb_platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t lps22hb_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void LPS22HB_init();
void LPS22HB_dataRead();

#endif // INC_LPS22HB_INTERFACE_H_
