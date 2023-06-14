/*
 * dichiarazione di tutte le variabili globali
 *
 */
#ifndef INC_GLOBAL_VARIABLES_H_
#define INC_GLOBAL_VARIABLES_H_

typedef struct data3xFloat lsm303agr_data;
typedef struct data3xFloat lsm6dsl_data;
typedef struct data3xFloat acc_mean;
struct data3xFloat {                          //struttura per i dati tridimensionali
	float x;
	float y;
	float z;
};

extern float pressure;

extern lsm303agr_data magnetometer;
extern lsm303agr_data accellerometer1;

extern lsm6dsl_data gyroscope;
extern lsm6dsl_data accellerometer2;

extern acc_mean accellerometer_mean;

#endif // INC_GLOBAL_VARIABLES_H_
