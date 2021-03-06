/*
 * Copyright (c) 2017 Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Board configuration macros for the efm32pg12b soc
 *
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE

#include <em_bus.h>
#include <em_common.h>
#include <device.h>

#include "soc_pinmap.h"
#include "../common/soc_gpio.h"

#endif /* !_ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* _SOC__H_ */
