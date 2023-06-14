/*
 * header per la dichiarazione di semafori globali
 */

#ifndef BIN_SEMAPHORES_H
#define BIN_SEMAPHORES_H

#include "cmsis_os.h"

extern osSemaphoreId_t binSemaphoreAcc1Handle;
extern osSemaphoreId_t binSemaphoreAcc2Handle;
extern osSemaphoreId_t binSemaphoreMagHandle;
extern osSemaphoreId_t binSemaphoreGyrHandle;
extern osSemaphoreId_t binSemaphorePresHandle;
extern osSemaphoreId_t binSemaphoreI2CHandle;
extern osSemaphoreId_t binSemaphoreAccMeanHandle;

#endif  // BIN_SEMAPHORES_H
