/*
 * Copyright (c) 2017-2018 MUS s.c.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_RTC_LEVEL
#include <logging/sys_log.h>

#include <misc/util.h>
#include <kernel.h>
#include <board.h>
#include <errno.h>
#include <rtc.h>
#include <power.h>

#include <clock_control/stm32_clock_control.h>
#include <clock_control.h>

#include <rtc_ll_stm32.h>

#define RTC_STM32_IRQ RTC_Alarm_IRQn

typedef struct
{
	RTC_TypeDef regs;
} rtc_stm32_ctx_t;

typedef struct
{
#ifdef CONFIG_RTC_STM32_API_REENTRANCY
	struct k_sem sem;
#endif
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	u32_t device_power_state;
#endif
	rtc_stm32_ctx_t context;
} rtc_stm32_data_t;

#define RTC_STM32_HAS_DATA \
	(CONFIG_RTC_STM32_API_REENTRANCY || CONFIG_DEVICE_POWER_MANAGEMENT)

#if RTC_STM32_HAS_DATA

static rtc_stm32_data_t rtc_stm32_data;
#define RTC_STM32_DATA (&rtc_stm32_data)

#define RTC_STM32_DEV_DATA_GET(dev) \
	((rtc_stm32_data_t * const)((dev)->driver_data))

#define RTC_STM32_DEV_CONFIG_GET(dev) \
	((rtc_stm32_config_t * const)((dev)->config->config_info))

int rtc_stm32_context_save(rtc_stm32_ctx_t *const ctx)
{
#if 0
	QM_CHECK(rtc < QM_SPI_NUM, -EINVAL);
	QM_CHECK(ctx != NULL, -EINVAL);

	qm_rtc_reg_t *const regs = QM_SPI[rtc];

	ctx->ctrlr0 = regs->ctrlr0;
	ctx->ser = regs->ser;
	ctx->baudr = regs->baudr;
#endif
	return 0;
}

int rtc_stm32_context_restore(const rtc_stm32_ctx_t *const ctx)
{
#if 0
	QM_CHECK(rtc < QM_SPI_NUM, -EINVAL);
	QM_CHECK(ctx != NULL, -EINVAL);

	qm_rtc_reg_t *const regs = QM_SPI[rtc];

	regs->ctrlr0 = ctx->ctrlr0;
	regs->ser = ctx->ser;
	regs->baudr = ctx->baudr;
#endif
	return 0;
}

#else

#define RTC_STM32_DATA (NULL)

#endif /* RTC_STM32_HAS_DATA */

#ifdef CONFIG_RTC_STM32_API_REENTRANCY
#define RTC_STM32_SEM_GET(dev) (&RTC_STM32_DEV_DATA_GET(dev)->sem)
#else
#define RTC_STM32_SEM_GET(dev) (NULL)
#endif

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT

static void rtc_stm32_power_state_set(struct device *dev, u32_t power_state)
{
	rtc_stm32_data_t *data = RTC_STM32_DEV_DATA_GET(dev);

	data->device_power_state = power_state;
}

static u32_t rtc_stm32_power_state_get(struct device *dev)
{
	rtc_stm32_data_t *data = RTC_STM32_DEV_DATA_GET(dev);

	return data->device_power_state;
}

#else

#define rtc_stm32_power_state_set(...)
#define rtc_stm32_power_state_get(...) DEVICE_PM_ACTIVE_STATE

#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */

#ifdef CONFIG_RTC_STM32_INTERRUPT
static void rtc_stm32_isr(void *arg)
{
	struct device * const dev = (struct device *)arg;
	rtc_stm32_data_t *data __attribute__((unused)) = RTC_STM32_DEV_DATA_GET(dev);
	rtc_stm32_config_t *cfg = RTC_STM32_DEV_CONFIG_GET(dev);
	RTC_TypeDef *rtc __attribute__((unused)) = cfg->rtc;
#if 0
	int err;

	err = rtc_stm32_get_err(rtc);
	if (err)
	{
		rtc_stm32_complete(data, rtc, err);
		return;
	}
#endif
}
#endif /* CONFIG_RTC_STM32_INTERRUPT */

#ifdef CONFIG_RTC_STM32_INTERRUPT

static void rtc_stm32_irq_config_set(struct device *dev);

#else

#define rtc_stm32_irq_config_set(...)

#endif /* CONFIG_RTC_STM32_INTERRUPT */

static int rtc_stm32_init(struct device *dev)
{
	rtc_stm32_data_t *data __attribute__((unused)) = RTC_STM32_DEV_DATA_GET(dev);
	rtc_stm32_config_t *cfg = RTC_STM32_DEV_CONFIG_GET(dev);

	__ASSERT_NO_MSG(device_get_binding(STM32_CLOCK_CONTROL_NAME));

	clock_control_on(device_get_binding(STM32_CLOCK_CONTROL_NAME),
			 (clock_control_subsys_t)&cfg->pclken);

	if (IS_ENABLED(CONFIG_RTC_STM32_API_REENTRANCY))
	{
		k_sem_init(RTC_STM32_SEM_GET(dev), 1, UINT_MAX);
	}
	rtc_stm32_irq_config_set(dev);

	rtc_stm32_power_state_set(dev, DEVICE_PM_ACTIVE_STATE);

	return 0;
}

static rtc_stm32_config_t rtc_stm32_cfg =
{
	.rtc = RTC,
	.pclken =
	{
		.enr = RCC_BDCR_RTCEN,
		.bus = STM32_CLOCK_BUS_APB1
	},
#ifdef CONFIG_RTC_STM32_INTERRUPT
	.irq_config = rtc_stm32_irq_config_set
#endif
};

static void rtc_stm32_enable(struct device *dev)
{
//	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);
}

static void rtc_stm32_disable(struct device *dev)
{
//	clk_periph_disable(CLK_PERIPH_RTC_REGISTER);
}


static int rtc_stm32_config_set(struct device *dev, struct rtc_config *cfg)
{
	int result = 0;
#if 0
	qm_rtc_config_t qm_cfg;

	qm_cfg.init_val = cfg->init_val;
	qm_cfg.alarm_en = cfg->alarm_enable;
	qm_cfg.alarm_val = cfg->alarm_val;

	/* Casting callback type due different input parameter from QMSI
	 * compared aganst the Zephyr callback from void cb(struct device *dev)
	 * to void cb(void *)
	 */
	qm_cfg.callback = (void *)cfg->cb_fn;
	qm_cfg.callback_data = dev;

	/* Set prescaler value. Ideally, the divider should come from struct
	 * rtc_config instead. It's safe to use RTC_DIVIDER here for now since
	 * values defined by clk_rtc_div and by QMSI's clk_rtc_div_t match for
	 * both D2000 and SE.
	 */
	qm_cfg.prescaler = (clk_rtc_div_t)RTC_DIVIDER;

	if (IS_ENABLED(CONFIG_RTC_STM32_API_REENTRANCY))
	{
		k_sem_take(RTC_STM32_SEM_GET(dev), K_FOREVER);
	}
	if (qm_rtc_set_config(QM_RTC_0, &qm_cfg))
	{
		result = -EIO;
	}
#endif
	if (IS_ENABLED(CONFIG_RTC_STM32_API_REENTRANCY))
	{
		k_sem_give(RTC_STM32_SEM_GET(dev));
	}

	k_busy_wait(60);

	return result;
}

static int rtc_stm32_alarm_set(struct device *dev, const u32_t alarm_val)
{
//	return qm_rtc_set_alarm(QM_RTC_0, alarm_val);
	return 0;
}

static u32_t rtc_stm32_read(struct device *dev)
{
//	return QM_RTC[QM_RTC_0]->rtc_ccvr;
	return 0;
}

static u32_t rtc_stm32_pending_int_get(struct device *dev)
{
//	return QM_RTC[QM_RTC_0]->rtc_stat;
	return 0;
}

static const struct rtc_driver_api rtc_stm32_api =
{
        .enable = rtc_stm32_enable,
        .disable = rtc_stm32_disable,
        .read = rtc_stm32_read,
        .set_config = rtc_stm32_config_set,
        .set_alarm = rtc_stm32_alarm_set,
        .get_pending_int = rtc_stm32_pending_int_get,
};

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT

static int rtc_stm32_device_suspend(struct device *dev)
{
	rtc_stm32_context_save(&RTC_STM32_DEV_DATA_GET(dev)->context);

	rtc_stm32_power_state_set(dev, DEVICE_PM_SUSPEND_STATE);

	return 0;
}

static int rtc_stm32_device_resume(struct device *dev)
{
	rtc_stm32_context_restore(&RTC_STM32_DEV_DATA_GET(dev)->context);

	rtc_stm32_power_state_set(dev, DEVICE_PM_ACTIVE_STATE);

	return 0;
}

/*
* Implements the driver control management functionality
* the *context may include IN data or/and OUT data
*/
static int rtc_stm32_device_ctrl(struct device *dev, u32_t cmd, void *context)
{
	u32_t *ctx = (u32_t *)context;
	if (cmd == DEVICE_PM_SET_POWER_STATE)
	{
		if (*ctx == DEVICE_PM_SUSPEND_STATE)
		{
			return rtc_stm32_device_suspend(dev);
		}
		else if (*ctx == DEVICE_PM_ACTIVE_STATE)
		{
			return rtc_stm32_device_resume(dev);
		}
	}
	else if (cmd == DEVICE_PM_GET_POWER_STATE)
	{
		*ctx = rtc_stm32_power_state_get(dev);
		return 0;
	}
	return 0;
}
#endif

DEVICE_DEFINE(rtc_stm32, CONFIG_RTC_0_NAME, &rtc_stm32_init,
	      rtc_stm32_device_ctrl, RTC_STM32_DATA, &rtc_stm32_cfg,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &rtc_stm32_api);

#ifdef CONFIG_RTC_STM32_INTERRUPT

static void rtc_stm32_irq_config_set(struct device *dev)
{
	IRQ_CONNECT(RTC_STM32_IRQ, CONFIG_RTC_STM32_IRQ_PRIORITY,
		    rtc_stm32_isr, DEVICE_GET(rtc_stm32), 0);

	irq_enable(RTC_STM32_IRQ);
}

#endif /* CONFIG_RTC_STM32_INTERRUPT */
