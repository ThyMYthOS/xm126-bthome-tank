// Copyright (c) Acconeer AB, 2023
// All rights reserved

#ifndef ACC_PM_H_
#define ACC_PM_H_

#include <stddef.h>

/**@brief Function for suspending a given uart device
 *
 * @param[in] uart The uart device to suspend
 * @return 0 if uart was suspended successfully
 */
int acc_pm_suspend_uart(const struct device *uart);

#endif
