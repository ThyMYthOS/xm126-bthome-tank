// Copyright (c) Acconeer AB, 2023
// All rights reserved

#ifndef ACC_BATTERY_INFO_H_
#define ACC_BATTERY_INFO_H_

#include <stdbool.h>

/**
 * @brief Configure the ADC for sampling the supply voltage
 *
 * @return true if initialization was successful, false otherwise
 */
bool acc_battery_info_init(void);


/**
 * @brief Sample the supply voltage
 *
 * @param[out] voltage      A sample of the supply voltage in units of volts.
 *                          Resolution is approximately 5.3 mV on XM122.
 * @param[in]  recalibrate  A Boolean value indicating whether the ADC should be
 *                          (re)calibrated before sampling.
 *
 * @return True if sampling was successful
 */
bool acc_battery_info_sample_voltage(float *voltage, bool recalibrate);


#endif
