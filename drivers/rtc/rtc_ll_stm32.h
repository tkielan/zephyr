/*
 * Copyright (c) 2017 MUS s.c.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_RTC_H_
#define _STM32_RTC_H_

typedef void (*rtc_stm32_irq_config)(struct device *dev);

typedef struct
{
	struct stm32_pclken pclken;
	RTC_TypeDef *rtc;
#ifdef CONFIG_RTC_STM32_INTERRUPT
	rtc_stm32_irq_config irq_config;
#endif
} rtc_stm32_config_t;

#endif	/* _STM32_RTC_H_ */
