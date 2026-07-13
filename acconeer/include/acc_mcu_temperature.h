// Copyright (c) Acconeer AB, 2023
// All rights reserved

#ifndef ACC_MCU_TEMPERATURE_H_
#define ACC_MCU_TEMPERATURE_H_

#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Function for reading the MCU die temperature
 *
 * @param[out] temperature The read die temperature
 * @return true if successful, false otherwise
 */
bool acc_mcu_temperature_read_die_temp(float *temperature);


#endif
