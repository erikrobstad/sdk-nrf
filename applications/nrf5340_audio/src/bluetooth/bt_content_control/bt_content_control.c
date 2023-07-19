/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_content_control.h"

#include <zephyr/zbus/zbus.h>

#include "bt_media_control.h"
#include "nrf5340_audio_common.h"
#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_content_control, CONFIG_BT_CONTENT_CONTROL_LOG_LEVEL);

ZBUS_CHAN_DEFINE(cont_media_chan, struct content_control_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

static void media_control_cb(bool play)
{
	int ret;
	struct content_control_msg msg;

	if (play) {
		msg.event = MEDIA_PLAY;
	} else {
		msg.event = MEDIA_PAUSE;
	}

	ret = zbus_chan_pub(&cont_media_chan, &msg, K_NO_WAIT);
	ERR_CHK_MSG(ret, "zbus publication failed");
}

int bt_content_control_start(struct bt_conn *conn)
{
	int ret;
	struct content_control_msg msg;

	if (IS_ENABLED(CONFIG_BT_MCC) || IS_ENABLED(CONFIG_BT_MCS)) {
		ret = bt_media_control_play(conn);
		if (ret) {
			LOG_WRN("Failed to change streaming state");
			return ret;
		}

		return 0;
	}

	msg.event = MEDIA_PLAY;

	ret = zbus_chan_pub(&cont_media_chan, &msg, K_NO_WAIT);
	ERR_CHK_MSG(ret, "zbus publication failed");

	return 0;
}

int bt_content_control_stop(struct bt_conn *conn)
{
	int ret;
	struct content_control_msg msg;

	if (IS_ENABLED(CONFIG_BT_MCC) || IS_ENABLED(CONFIG_BT_MCS)) {
		ret = bt_media_control_pause(conn);
		if (ret) {
			LOG_WRN("Failed to change streaming state");
			return ret;
		}

		return 0;
	}

	msg.event = MEDIA_PAUSE;

	ret = zbus_chan_pub(&cont_media_chan, &msg, K_NO_WAIT);
	ERR_CHK_MSG(ret, "zbus publication failed");

	return 0;
}

int bt_content_control_conn_disconnected(struct bt_conn *conn)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = bt_media_control_conn_disconnected(conn);
		if (ret) {
			LOG_ERR("bt_media_control_conn_disconnected failed with %d", ret);
		}
	}

	return 0;
}

int bt_content_control_discover(struct bt_conn *conn)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = bt_media_control_discover(conn);
		if (ret) {
			LOG_ERR("Failed to discover media control client");
			return ret;
		}
	}

	return 0;
}

int bt_content_control_init(void)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_MCS)) {
		ret = bt_media_control_server_init(media_control_cb);
		if (ret) {
			LOG_ERR("MCS server init failed");
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = bt_media_control_client_init();
		if (ret) {
			LOG_ERR("MCS client init failed");
			return ret;
		}
	}

	return 0;
}
