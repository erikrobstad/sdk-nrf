/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_rend.h"

#include <zephyr/zbus/zbus.h>

#include "bt_volume.h"
#include "nrf5340_audio_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_renderer, CONFIG_BT_RENDERER_LOG_LEVEL);

ZBUS_CHAN_DEFINE(volume_chan, struct volume_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

int bt_rend_volume_up(void)
{
	int ret;
	struct volume_msg msg;

	if (IS_ENABLED(CONFIG_BT_VCP_VOL_CTLR) || IS_ENABLED(CONFIG_BT_VCP_VOL_REND)) {
		ret = bt_vol_up();
		return ret;
	}

	msg.event = VOLUME_UP;

	ret = zbus_chan_pub(&volume_chan, &msg, K_NO_WAIT);
	return ret;
}

int bt_rend_volume_down(void)
{
	int ret;
	struct volume_msg msg;

	if (IS_ENABLED(CONFIG_BT_VCP_VOL_CTLR) || IS_ENABLED(CONFIG_BT_VCP_VOL_REND)) {
		ret = bt_vol_down();
		return ret;
	}

	msg.event = VOLUME_DOWN;

	ret = zbus_chan_pub(&volume_chan, &msg, K_NO_WAIT);
	return ret;
}

int bt_rend_volume_set(uint8_t volume, bool from_vcp)
{
	int ret;
	struct volume_msg msg;

	if ((IS_ENABLED(CONFIG_BT_VCP_VOL_CTLR) || IS_ENABLED(CONFIG_BT_VCP_VOL_REND)) &&
	    !from_vcp) {
		ret = bt_vol_set(volume);
		return ret;
	}

	msg.event = VOLUME_SET;
	msg.volume = volume;

	ret = zbus_chan_pub(&volume_chan, &msg, K_NO_WAIT);
	return ret;
}

int bt_rend_mute(bool from_vcp)
{
	int ret;
	struct volume_msg msg;

	if ((IS_ENABLED(CONFIG_BT_VCP_VOL_CTLR) || IS_ENABLED(CONFIG_BT_VCP_VOL_REND)) &&
	    !from_vcp) {
		ret = bt_vol_mute();
		return ret;
	}

	msg.event = VOLUME_MUTE;

	ret = zbus_chan_pub(&volume_chan, &msg, K_NO_WAIT);
	return ret;
}

int bt_rend_unmute(void)
{
	int ret;
	struct volume_msg msg;

	if (IS_ENABLED(CONFIG_BT_VCP_VOL_CTLR) || IS_ENABLED(CONFIG_BT_VCP_VOL_REND)) {
		ret = bt_vol_unmute();
		return ret;
	}

	msg.event = VOLUME_UNMUTE;

	ret = zbus_chan_pub(&volume_chan, &msg, K_NO_WAIT);
	return ret;
}

int bt_rend_discover(struct bt_conn *conn)
{
	int ret;

	/* Only do a VCS discover if we are volume controller */
	if (IS_ENABLED(CONFIG_BT_VCP_VOL_CTLR)) {
		ret = bt_vol_vcs_discover(conn);
		if (ret) {
			LOG_WRN("Failed to discover VCS: %d", ret);
			return ret;
		}
	} else {
		LOG_WRN("VCS controller not enabled");
	}

	return 0;
}

int bt_rend_init(void)
{
	int ret;
	bool cfgs_enabled = false;

	if (IS_ENABLED(CONFIG_BT_VCP_VOL_CTLR)) {
		ret = bt_vol_vcs_ctlr_init();

		if (ret) {
			LOG_WRN("Failed to initialize VCS controller: %d", ret);
			return ret;
		}

		cfgs_enabled = true;
	}

	if (IS_ENABLED(CONFIG_BT_VCP_VOL_REND)) {
		ret = bt_vol_vcs_rend_init();

		if (ret) {
			LOG_WRN("Failed to initialize VCS renderer: %d", ret);
			return ret;
		}

		cfgs_enabled = true;
	}

	if (!cfgs_enabled) {
		LOG_WRN("Both VCS controller and VCS renderer are disabled");
	}

	return 0;
}
