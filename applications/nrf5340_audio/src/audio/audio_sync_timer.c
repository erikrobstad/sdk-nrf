/*************************************************************************************************/
/*
 *  Copyright (c) 2021, PACKETCRAFT, INC.
 *  All rights reserved.
 */
/*************************************************************************************************/

/*
 * Redistribution and use of the Audio subsystem for nRF5340 Software, in binary
 * and source code forms, with or without modification, are permitted provided
 * that the following conditions are met:
 *
 * 1. Redistributions of source code form must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer.
 *
 * 2. Redistributions in binary code form, except as embedded into a Nordic
 *    Semiconductor ASA nRF53 chip or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions
 *    and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of Packetcraft, Inc. nor Nordic Semiconductor ASA nor
 *    the names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA nRF53 chip.
 *
 * 5. Any software provided in binary or source code form under this license
 *    must not be reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY PACKETCRAFT, INC. AND NORDIC SEMICONDUCTOR ASA
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL PACKETCRAFT, INC.,
 * NORDIC SEMICONDUCTOR ASA, OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "audio_sync_timer.h"

#include <zephyr.h>
#include <zephyr/kernel.h>
#include <init.h>
#include <nrfx_timer.h>
#include <nrfx_dppi.h>
#include <nrfx_i2s.h>
#include <nrfx_ipc.h>
#include <nrfx_gpiote.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_sync_timer, CONFIG_LOG_AUDIO_SYNC_TIMER_LEVEL);

#define AUDIO_SYNC_TIMER_INSTANCE 1

#define AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL 0
#define AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE NRF_TIMER_TASK_CAPTURE0

#define AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL 1

#define AUDIO_SYNC_TIMER_SYNC_LED_ON_CMPR_CHANNEL 2
#define AUDIO_SYNC_TIMER_SYNC_LED_ON_CMPR NRF_TIMER_EVENT_COMPARE2

#define AUDIO_SYNC_TIMER_SYNC_LED_OFF_CMPR_CHANNEL 3
#define AUDIO_SYNC_TIMER_SYNC_LED_OFF_CMPR NRF_TIMER_EVENT_COMPARE3

#define AUDIO_SYNC_TIMER_NET_APP_IPC_EVT NRF_IPC_EVENT_RECEIVE_4

#define AUDIO_SYNC_TIMER_IRQ_PRIORITY 6

#define SYNC_LED DT_GPIO_PIN(DT_NODELABEL(led1_blue), gpios)

static const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(AUDIO_SYNC_TIMER_INSTANCE);

static uint8_t dppi_channel_timer_clear;
static uint8_t dppi_channel_i2s_frame_start;
static uint8_t dppi_channel_gpiote_clear;
static uint8_t dppi_channel_gpiote_set;

static nrfx_timer_config_t cfg = { .frequency = NRF_TIMER_FREQ_1MHz,
				   .mode = NRF_TIMER_MODE_TIMER,
				   .bit_width = NRF_TIMER_BIT_WIDTH_32,
				   .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
				   .p_context = NULL };

static void event_handler(nrf_timer_event_t event_type, void *ctx)
{
}

static int audio_sync_timer_sync_led_init(void)
{
	nrfx_err_t ret;

	/* Connect the IRQ handler for timer 1 */
	if (AUDIO_SYNC_TIMER_INSTANCE == 1) {
		IRQ_CONNECT(DT_IRQN(DT_NODELABEL(timer1)), AUDIO_SYNC_TIMER_IRQ_PRIORITY, nrfx_isr,
			    nrfx_timer_1_irq_handler, 0);
	} else {
		LOG_WRN("Invalid audio sync timer instance");
		return -ECANCELED;
	}

	if (nrfx_gpiote_is_init()) {
		LOG_DBG("GPIOTE is already initiated");
	} else {
		nrfx_gpiote_init(0);
		LOG_DBG("GPIOTE initialized");
	}

	/* GPIOTE out configs for sync LED */
	nrfx_gpiote_out_config_t const gpiote_cfg = NRFX_GPIOTE_CONFIG_OUT_TASK_HIGH;
	ret = nrfx_gpiote_out_init(SYNC_LED, &gpiote_cfg);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx GPIOTE init error - Return value: %d", ret);
		return ret;
	}

	nrfx_gpiote_out_task_enable(SYNC_LED);

	/* Initializing SET task for GPIOTE (sync LED) */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_gpiote_set);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (GPIOTE SET task) - Return value: %d", ret);
		return ret;
	}

	nrf_gpiote_task_t gpiote_set_task = nrfx_gpiote_set_task_get(SYNC_LED);
	nrf_timer_publish_set(timer_instance.p_reg, AUDIO_SYNC_TIMER_SYNC_LED_ON_CMPR,
			      dppi_channel_gpiote_set);
	nrf_gpiote_subscribe_set(NRF_GPIOTE, gpiote_set_task, dppi_channel_gpiote_set);
	ret = nrfx_dppi_channel_enable(dppi_channel_gpiote_set);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (GPIOTE SET task) - Return value: %d", ret);
		return ret;
	}

	/* Initializing CLEAR task for GPIOTE (sync LED) */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_gpiote_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (GPIOTE CLR task) - Return value: %d", ret);
		return ret;
	}

	nrf_gpiote_task_t gpiote_clear_task = nrfx_gpiote_clr_task_get(SYNC_LED);
	nrf_timer_publish_set(timer_instance.p_reg, AUDIO_SYNC_TIMER_SYNC_LED_OFF_CMPR,
			      dppi_channel_gpiote_clear);
	nrf_gpiote_subscribe_set(NRF_GPIOTE, gpiote_clear_task, dppi_channel_gpiote_clear);
	ret = nrfx_dppi_channel_enable(dppi_channel_gpiote_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (GPIOTE CLR task) - Return value: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Initialize audio sync timer
 *
 * @note Clearing of the nRF5340 APP core sync
 * timer is initialized here. The sync timers on
 * APP core and NET core are cleared at exactly
 * the same time using an IPC signal sent from
 * the NET core. This makes the two timers
 * synchronized.
 *
 * @param unused Unused
 *
 * @return 0 if successful, error otherwise
 */
static int audio_sync_timer_init(const struct device *unused)
{
	nrfx_err_t ret;

	ARG_UNUSED(unused);

	ret = nrfx_timer_init(&timer_instance, &cfg, event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx timer init error - Return value: %d", ret);
		return ret;
	}

	ret = audio_sync_timer_sync_led_init();
	if (ret) {
		LOG_ERR("Error setting up LED for sync - Return value: %d", ret);
		return ret;
	}

	nrfx_timer_enable(&timer_instance);

	/* Initialize capturing of I2S frame start event timestamps */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_i2s_frame_start);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (I2S frame start) - Return value: %d", ret);
		return ret;
	}
	nrf_timer_subscribe_set(timer_instance.p_reg, AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE,
				dppi_channel_i2s_frame_start);
	nrf_i2s_publish_set(NRF_I2S0, NRF_I2S_EVENT_FRAMESTART, dppi_channel_i2s_frame_start);
	ret = nrfx_dppi_channel_enable(dppi_channel_i2s_frame_start);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (I2S frame start) - Return value: %d", ret);
		return ret;
	}

	/* Initialize functionality for synchronization between APP and NET core */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (timer clear) - Return value: %d", ret);
		return ret;
	}
	nrf_ipc_publish_set(NRF_IPC, AUDIO_SYNC_TIMER_NET_APP_IPC_EVT, dppi_channel_timer_clear);
	nrf_timer_subscribe_set(timer_instance.p_reg, NRF_TIMER_TASK_CLEAR,
				dppi_channel_timer_clear);
	ret = nrfx_dppi_channel_enable(dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (timer clear) - Return value: %d", ret);
		return ret;
	}

	LOG_INF("Audio sync timer initialized");

	return 0;
}

uint32_t audio_sync_timer_i2s_frame_start_ts_get(void)
{
	return nrfx_timer_capture_get(&timer_instance,
				      AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL);
}

uint32_t audio_sync_timer_curr_time_get(void)
{
	return nrfx_timer_capture(&timer_instance, AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);
}

void audio_sync_timer_sync_led_on()
{
	nrfx_timer_compare(&timer_instance, AUDIO_SYNC_TIMER_SYNC_LED_OFF_CMPR_CHANNEL, 0, true);
	nrfx_gpiote_set_task_trigger(SYNC_LED);
}

void audio_sync_timer_sync_led_cmpr_time_set(uint32_t compare_value, bool clear)
{
	if (clear) {
		nrfx_timer_compare(&timer_instance, AUDIO_SYNC_TIMER_SYNC_LED_OFF_CMPR_CHANNEL,
				   compare_value, true);
	} else {
		nrfx_timer_compare(&timer_instance, AUDIO_SYNC_TIMER_SYNC_LED_ON_CMPR_CHANNEL,
				   compare_value, true);
	}
}

SYS_INIT(audio_sync_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
