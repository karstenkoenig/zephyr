/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 * Copyright (c) 2016 Cadence Design Systems, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Xtensa public exception handling
 *
 * Xtensa-specific kernel exception handling interface. Included by
 * arch/xtensa/arch.h.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_XTENSA_EXC_H_
#define ZEPHYR_INCLUDE_ARCH_XTENSA_EXC_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE
/**
 * @brief Exception Stack Frame
 *
 * A pointer to an "exception stack frame" (ESF) is passed as an argument
 * to exception handlers registered via nanoCpuExcConnect().
 */
struct __esf {
	/* FIXME - not finished yet */
	sys_define_gpr_with_alias(a1, sp);
	u32_t pc;
};

typedef struct __esf z_arch_esf_t;
#endif

#ifdef __cplusplus
}
#endif


#endif /* ZEPHYR_INCLUDE_ARCH_XTENSA_EXC_H_ */
