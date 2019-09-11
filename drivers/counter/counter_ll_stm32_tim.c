/*
 * Copyright (c) 2018 Workaround GmbH
 * Copyright (c) 2018 Allterco Robotics
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Source file for the STM32 RTC driver
 *
 */

#include <time.h>

#include <clock_control/stm32_clock_control.h>
#include <clock_control.h>
#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <counter.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(counter_tim_stm32, CONFIG_COUNTER_LOG_LEVEL);

#define T_TIME_OFFSET 946684800

struct tim_stm32_config {
	struct counter_config_info counter_info;
	struct stm32_pclken pclken;
	LL_TIM_InitTypeDef ll_tim_config;
	TIM_TypeDef *timer;
};

struct tim_stm32_ch_data {
	counter_alarm_callback_t callback;
	u32_t ticks;
	void *user_data;
	bool absolute;
};

struct tim_stm32_data {
	struct tim_stm32_ch_data *ch_data;
};


#define DEV_DATA(dev) ((struct tim_stm32_data *)(dev)->driver_data)
#define DEV_CFG(dev)	\
((struct tim_stm32_config * const)(dev)->config->config_info)


static int tim_stm32_start(struct device *dev)
{
	const struct tim_stm32_config *cfg = DEV_CFG(dev);

	LL_TIM_EnableCounter(cfg->timer);

	return 0;
}


static int tim_stm32_stop(struct device *dev)
{
	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	
	LL_TIM_DisableCounter(cfg->timer);

	return 0;
}


static u32_t tim_stm32_read(struct device *dev)
{
	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	
	return LL_TIM_GetCounter(cfg->timer);
}

static int tim_stm32_set_alarm(struct device *dev, u8_t chan_id,
				const struct counter_alarm_cfg *alarm_cfg)
{
	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	struct tim_stm32_data *data = DEV_DATA(dev);
	
	u32_t now = LL_TIM_GetCounter(cfg->timer);
	u32_t ticks = alarm_cfg->ticks;

	if(chan_id >= cfg->counter_info.channels) {
		return -EINVAL;
	}

	if (alarm_cfg->ticks > LL_TIM_GetAutoReload(cfg->timer)) {
		return -EINVAL;
	}

	if (data->ch_data[chan_id].callback != NULL) {
		LOG_DBG("Alarm busy\n");
		return -EBUSY;
	}
	
	data->ch_data[chan_id].callback = alarm_cfg->callback;
	data->ch_data[chan_id].user_data = alarm_cfg->user_data;
	data->ch_data[chan_id].absolute = alarm_cfg->absolute;

	if (!alarm_cfg->absolute) {
		ticks += now;
	}

	data->ch_data[chan_id].ticks = ticks;

	LOG_DBG("Set Alarm: %d\n", ticks);

	switch(chan_id) {
		case 0: 
			LL_TIM_OC_SetCompareCH1(cfg->timer, ticks);
			LL_TIM_ClearFlag_CC1(cfg->timer);
			LL_TIM_EnableIT_CC1(cfg->timer);
			LL_TIM_CC_EnableChannel(cfg->timer, LL_TIM_CHANNEL_CH1);
			break;
		case 1: 
			LL_TIM_OC_SetCompareCH2(cfg->timer, ticks);
			LL_TIM_ClearFlag_CC2(cfg->timer);
			LL_TIM_EnableIT_CC2(cfg->timer);
			LL_TIM_CC_EnableChannel(cfg->timer, LL_TIM_CHANNEL_CH2);
			break;
		case 2: 
			LL_TIM_OC_SetCompareCH3(cfg->timer, ticks);
			LL_TIM_ClearFlag_CC3(cfg->timer);
			LL_TIM_EnableIT_CC3(cfg->timer);
			LL_TIM_CC_EnableChannel(cfg->timer, LL_TIM_CHANNEL_CH3);
			break;
		case 3: 
			LL_TIM_OC_SetCompareCH4(cfg->timer, ticks);
			LL_TIM_ClearFlag_CC4(cfg->timer);
			LL_TIM_EnableIT_CC4(cfg->timer);
			LL_TIM_CC_EnableChannel(cfg->timer, LL_TIM_CHANNEL_CH4);
			break;
		default: break;
	}

	return 0;
}


static int tim_stm32_cancel_alarm(struct device *dev, u8_t chan_id)
{
	const struct tim_stm32_config *cfg = DEV_CFG(dev);

	if(chan_id >= cfg->counter_info.channels) {
		return -EINVAL;
	}

	switch(chan_id) {
		case 0: 
			LL_TIM_ClearFlag_CC1(cfg->timer);
			LL_TIM_DisableIT_CC1(cfg->timer);
			LL_TIM_CC_DisableChannel(cfg->timer, LL_TIM_CHANNEL_CH1);
			break;
		case 1: 
			LL_TIM_ClearFlag_CC2(cfg->timer);
			LL_TIM_DisableIT_CC2(cfg->timer);
			LL_TIM_CC_DisableChannel(cfg->timer, LL_TIM_CHANNEL_CH2);
			break;
		case 2: 
			LL_TIM_ClearFlag_CC3(cfg->timer);
			LL_TIM_DisableIT_CC3(cfg->timer);
			LL_TIM_CC_DisableChannel(cfg->timer, LL_TIM_CHANNEL_CH3);
			break;
		case 3: 
			LL_TIM_ClearFlag_CC4(cfg->timer);
			LL_TIM_DisableIT_CC4(cfg->timer);
			LL_TIM_CC_DisableChannel(cfg->timer, LL_TIM_CHANNEL_CH4);
			break;
		default: break;
	}

	DEV_DATA(dev)->ch_data[chan_id].callback = NULL;

	return 0;
}


static u32_t tim_stm32_get_pending_int(struct device *dev)
{
	// struct device *const dev = (struct device *)arg;
	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	
	return (LL_TIM_IsActiveFlag_CC1(cfg->timer)) ||
	  	   (LL_TIM_IsActiveFlag_CC2(cfg->timer)) ||
		   (LL_TIM_IsActiveFlag_CC3(cfg->timer)) ||
		   (LL_TIM_IsActiveFlag_CC4(cfg->timer)) || 
		   (LL_TIM_IsActiveFlag_UPDATE(cfg->timer));

}


static u32_t tim_stm32_get_top_value(struct device *dev)
{
	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	
	return LL_TIM_GetAutoReload(cfg->timer);
}


static int tim_stm32_set_top_value(struct device *dev, u32_t ticks,
				counter_top_callback_t callback,
				void *user_data)
{

	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	
	LL_TIM_SetAutoReload(cfg->timer, ticks);

	return 0;	
}


static u32_t tim_stm32_get_max_relative_alarm(struct device *dev)
{
	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	
	return LL_TIM_GetAutoReload(cfg->timer);
}


void tim_stm32_isr(void *arg)
{
	struct device *const dev = (struct device *)arg;
	struct tim_stm32_data *data = DEV_DATA(dev);
	const struct tim_stm32_config *cfg = DEV_CFG(dev);
	

	u32_t now = tim_stm32_read(dev);

	if (LL_TIM_IsActiveFlag_CC1(cfg->timer)) {
		LL_TIM_ClearFlag_CC1(cfg->timer);
		if(data->ch_data[0].callback){
			counter_alarm_callback_t clbk  = data->ch_data[0].callback;
			data->ch_data[0].callback = NULL;
			clbk(dev, 0, now, data->ch_data[0].user_data);
		}
	}

	if (LL_TIM_IsActiveFlag_CC2(cfg->timer)) {
		LL_TIM_ClearFlag_CC2(cfg->timer);
		if(data->ch_data[1].callback){
			counter_alarm_callback_t clbk  = data->ch_data[1].callback;
			data->ch_data[1].callback = NULL;
			clbk(dev, 1, now, data->ch_data[1].user_data);
		}
	}
	
	if (LL_TIM_IsActiveFlag_CC3(cfg->timer)) {
		LL_TIM_ClearFlag_CC3(cfg->timer);
		if(data->ch_data[2].callback){
			counter_alarm_callback_t clbk  = data->ch_data[2].callback;
			data->ch_data[2].callback = NULL;
			clbk(dev, 2, now, data->ch_data[2].user_data);
		}
	}
	
	if (LL_TIM_IsActiveFlag_CC4(cfg->timer)) {
		LL_TIM_ClearFlag_CC4(cfg->timer);
		if(data->ch_data[3].callback){
			counter_alarm_callback_t clbk  = data->ch_data[3].callback;
			data->ch_data[3].callback = NULL;
			clbk(dev, 3, now, data->ch_data[3].user_data);
		}
	}
	
	if (LL_TIM_IsActiveFlag_UPDATE(cfg->timer)) {
		LL_TIM_ClearFlag_UPDATE(cfg->timer);
		// if(data->ch_data[4].callback){
		// 	counter_alarm_callback_t clbk  = data->ch_data[4].callback;
		// 	data->ch_data[4].callback = NULL;
		// 	clbk(data->ch_data[4].user_data);
		// }
	}
	
	//LL_EXTI_ClearFlag_0_31(RTC_EXTI_LINE);
}

static const struct counter_driver_api tim_stm32_driver_api = {
		.start = tim_stm32_start,
		.stop = tim_stm32_stop,
		.read = tim_stm32_read,
		.set_alarm = tim_stm32_set_alarm,
		.cancel_alarm = tim_stm32_cancel_alarm,
		.set_top_value = tim_stm32_set_top_value,
		.get_pending_int = tim_stm32_get_pending_int,
		.get_top_value = tim_stm32_get_top_value,
		.get_max_relative_alarm = tim_stm32_get_max_relative_alarm,
};

#define COUNTER_TIM_STM32_DEVICE(idx)	\
	static int tim##idx##_stm32_init(struct device *dev); \
	static struct tim_stm32_ch_data tim##idx##_ch_data[4]; \
	static struct tim_stm32_data tim##idx##_data; \
	static struct tim_stm32_config tim##idx##_config = { \
		.counter_info = { \
			.max_top_value = UINT32_MAX, \
			.freq = 1000000, \
			.count_up = true, \
			.channels = 4, \
		}, \
		.pclken = { \
			.enr = DT_TIMER_STM32_##idx##_CLOCK_BITS, \
			.bus = DT_TIMER_STM32_##idx##_CLOCK_BUS, \
		}, \
	}; \
	DEVICE_AND_API_INIT(tim##idx##_stm32, DT_TIMER_STM32_##idx##_LABEL, tim##idx##_stm32_init, \
				&tim##idx##_data, &tim##idx##_config, POST_KERNEL, \
				CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &tim_stm32_driver_api); \
	static int tim##idx##_stm32_init(struct device *dev) \
	{ \
		struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME); \
		struct tim_stm32_config *cfg = DEV_CFG(dev); \
		struct tim_stm32_data *data = DEV_DATA(dev); \
		data->ch_data = tim##idx##_ch_data; \
		for (int i = 0; i < cfg->counter_info.channels; i++)  \
			data->ch_data[i].callback = NULL; \
		if(clk) \
			clock_control_on(clk, (clock_control_subsys_t *) &cfg->pclken); \
		cfg->ll_tim_config.Prescaler = __LL_TIM_CALC_PSC(SystemCoreClock, 1000000); \
		cfg->ll_tim_config.CounterMode = LL_TIM_COUNTERMODE_UP; \
		cfg->ll_tim_config.Autoreload = UINT32_MAX; \
		cfg->ll_tim_config.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; \
		cfg->ll_tim_config.RepetitionCounter = (uint8_t)0x00; \
		cfg->timer = DT_TIMER_STM32_##idx##_BASE_ADDRESS; \
		if (LL_TIM_DeInit(cfg->timer) != SUCCESS) \
			return -EIO; \
		if (LL_TIM_Init(cfg->timer, ((LL_TIM_InitTypeDef *) \
					&cfg->ll_tim_config)) != SUCCESS)  \
			return -EIO; \
		IRQ_CONNECT(DT_TIMER_STM32_##idx##_IRQ, DT_TIMER_STM32_##idx##_IRQ_PRI, \
				tim_stm32_isr, DEVICE_GET(tim##idx##_stm32), 0); \
		irq_enable(DT_TIMER_STM32_##idx##_IRQ); \
		return 0; \
	} \

#ifdef CONFIG_COUNTER_TIM_STM32_TIMER0
COUNTER_TIM_STM32_DEVICE(0);
#endif

#ifdef CONFIG_COUNTER_TIM_STM32_TIMER1
COUNTER_TIM_STM32_DEVICE(1);
#endif

