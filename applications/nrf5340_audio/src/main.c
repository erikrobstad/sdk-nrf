/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/debug/stack.h>
#include <zephyr/device.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "fw_info_app.h"
#include "led.h"
#include "button_handler.h"
#include "button_assignments.h"
#include "nrfx_clock.h"
#include "ble_core.h"
#include "power_module.h"
#include "sd_card.h"
#include "board_version.h"
#include "audio_system.h"
#include "channel_assignment.h"
#include "streamctrl.h"
#include "audio_lc3.h"
#include "audio_i2s.h"
#include "hw_codec.h"
#include <zephyr/sys/ring_buffer.h>


#if defined(CONFIG_AUDIO_DFU_ENABLE)
#include "dfu_entry.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#if defined(CONFIG_INIT_STACKS)
/* Used for printing stack usage */
extern struct k_thread z_main_thread;
#endif /* defined(CONFIG_INIT_STACKS) */

static atomic_t ble_core_is_ready = (atomic_t) false;
static struct board_version board_rev;

static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

static int leds_set(void)
{
	int ret;

	/* Blink LED 3 to indicate that APP core is running */
	ret = led_blink(LED_APP_3_GREEN);
	if (ret) {
		return ret;
	}

#if (CONFIG_AUDIO_DEV == HEADSET)
	enum audio_channel channel;

	channel_assignment_get(&channel);

	if (channel == AUDIO_CH_L) {
		ret = led_on(LED_APP_RGB, LED_COLOR_BLUE);
	} else {
		ret = led_on(LED_APP_RGB, LED_COLOR_MAGENTA);
	}

	if (ret) {
		return ret;
	}
#elif (CONFIG_AUDIO_DEV == GATEWAY)
	ret = led_on(LED_APP_RGB, LED_COLOR_GREEN);
	if (ret) {
		return ret;
	}
#endif /* (CONFIG_AUDIO_DEV == HEADSET) */

	return 0;
}

static int bonding_clear_check(void)
{
	int ret;
	bool pressed;

	ret = button_pressed(BUTTON_5, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			LOG_INF("Clearing all bonds");
			bt_unpair(BT_ID_DEFAULT, NULL);
		}
	}
	return 0;
}

static int channel_assign_check(void)
{
#if (CONFIG_AUDIO_DEV == HEADSET) && CONFIG_AUDIO_HEADSET_CHANNEL_RUNTIME
	int ret;
	bool pressed;

	ret = button_pressed(BUTTON_VOLUME_DOWN, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		channel_assignment_set(AUDIO_CH_L);
		return 0;
	}

	ret = button_pressed(BUTTON_VOLUME_UP, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		channel_assignment_set(AUDIO_CH_R);
		return 0;
	}
#endif

	return 0;
}

/* Callback from ble_core when the ble subsystem is ready */
void on_ble_core_ready(void)
{
	int ret;

	(void)atomic_set(&ble_core_is_ready, (atomic_t) true);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();

		ret = bonding_clear_check();
		ERR_CHK(ret);
	}
}

void main(void)
{
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = hfclock_config_and_start();
	ERR_CHK(ret);

	ret = led_init();
	ERR_CHK(ret);

	ret = button_handler_init();
	ERR_CHK(ret);

	channel_assignment_init();

	ret = channel_assign_check();
	ERR_CHK(ret);

	ret = fw_info_app_print();
	ERR_CHK(ret);

	ret = board_version_valid_check();
	ERR_CHK(ret);

	ret = board_version_get(&board_rev);
	ERR_CHK(ret);

	if (board_rev.mask & BOARD_VERSION_VALID_MSK_SD_CARD) {
		ret = sd_card_init();
		if (ret != -ENODEV) {
			ERR_CHK(ret);
		}
	}

	ret = power_module_init();
	ERR_CHK(ret);

#if defined(CONFIG_AUDIO_DFU_ENABLE)
	/* Check DFU BTN before Initialize BLE */
	dfu_entry_check();
#endif

	/* Initialize BLE, with callback for when BLE is ready */
	ret = ble_core_init(on_ble_core_ready);
	ERR_CHK(ret);

	/* Wait until ble_core/NET core is ready */
	while (!(bool)atomic_get(&ble_core_is_ready)) {
		(void)k_sleep(K_MSEC(100));
	}

	ret = leds_set();
	ERR_CHK(ret);

#if defined(CONFIG_AUDIO_LC3)
	ret = audio_lc3_play_init();
	ERR_CHK(ret);
#endif

	// List all the files on the card
	// char buf11[1024] = {0};
	// printk("The size of buf11 is %zu\n", sizeof(buf11));
	// ret = sd_card_list_files("/", buf11, sizeof(buf11));
	// if(ret < 0) {
	// 	LOG_ERR("Unable to read files from SD card (err %i)", ret);
	// }

	audio_system_init();
	audio_i2s_blk_comp_cb_register(audio_lc3_i2s_callback);

	// Denne funksjonen funker ikke sammen med audio_wav
	// ret = streamctrl_start();
	// ERR_CHK(ret);

	// while (1) {
	// 	streamctrl_event_handler();
	// 	STACK_USAGE_PRINT("main", &z_main_thread);
	// }
	k_msleep(2000);
	ERR_CHK(ret);
	hw_codec_default_conf_enable();
	// audio_lc3_read_header_file("enc_4.bin", "");
	audio_lc3_play_init("enc_1.bin", "");
	audio_lc3_play("enc_1.bin", "");
}
