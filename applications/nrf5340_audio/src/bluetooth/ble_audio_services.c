/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_audio_services.h"

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/audio/media_proxy.h>
#include <zephyr/bluetooth/audio/mcs.h>
#include <zephyr/bluetooth/audio/mcc.h>

#include "macros_common.h"
#include "hw_codec.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble_audio_services, CONFIG_AUDIO_SERVICES_LOG_LEVEL);

static uint8_t media_player_state = BT_MCS_MEDIA_STATE_PLAYING;

static struct media_player *local_player;
static ble_mcs_play_pause_cb play_pause_cb;

static uint8_t mcp_mcs_disc_status[CONFIG_BT_MAX_CONN];

enum mcs_disc_status {
	IDLE,
	IN_PROGRESS,
	FINISHED,
};

/**
 * @brief  Callback handler for MCS discover finished.
 *
 *         This callback handler will be triggered when MCS
 *         discovery is finished. Used by the client.
 */
static void mcc_discover_mcs_cb(struct bt_conn *conn, int err)
{
	int ret;
	uint8_t idx = bt_conn_index(conn);

	if (err) {
		LOG_ERR("Discovery of MCS failed (%d)", err);
		mcp_mcs_disc_status[idx] = IDLE;
		return;
	}

	if (mcp_mcs_disc_status[idx] != IN_PROGRESS) {
		/* Due to the design of MCC, there will be several
		 * invocations of this callback. We are only interested
		 * in what we have explicitly requested.
		 */
		LOG_DBG("Filtered out callback");
		return;
	}

	mcp_mcs_disc_status[idx] = FINISHED;
	LOG_INF("Discovery of MCS finished");
	ret = ble_mcs_state_update(conn);
	if (ret < 0 && ret != -EBUSY) {
		LOG_WRN("Failed to update media state: %d", ret);
	}
}

/**
 * @brief  Callback handler for sent MCS commands.
 *
 *         This callback will be triggered when MCS commands have been sent.
 *         Only for debugging, used by the client.
 */
static void mcc_send_command_cb(struct bt_conn *conn, int err, const struct mpl_cmd *cmd)
{
	LOG_DBG("mcc_send_command_cb");

	if (err) {
		LOG_ERR("MCC: cmd send failed (%d) - opcode: %u, param: %d", err, cmd->opcode,
			cmd->param);
	}
}

/**
 * @brief  Callback handler for received notifications.
 *
 *         This callback will be triggered when a notification has been received.
 *         Only for debugging, used by the client.
 */
static void mcc_cmd_notification_cb(struct bt_conn *conn, int err, const struct mpl_cmd_ntf *ntf)
{
	LOG_DBG("mcc_cmd_ntf_cb");

	if (err) {
		LOG_ERR("MCC: cmd ntf error (%d) - opcode: %u, result: %u", err,
			ntf->requested_opcode, ntf->result_code);
	}
}

/**
 * @brief  Callback handler for reading media state.
 *
 *         This callback will be triggered when the client has asked to read
 *         the current state of the media player.
 */
static void mcc_read_media_state_cb(struct bt_conn *conn, int err, uint8_t state)
{
	LOG_DBG("mcc_read_media_cb, state: %d", state);

	if (err) {
		LOG_ERR("MCC: Media State read failed (%d)", err);
		return;
	}

	media_player_state = state;
}

/**
 * @brief  Callback handler for received MCS commands.
 *
 *         This callback will be triggered when the server has receieved a
 *         command from the client or the commander.
 */
static void mcs_command_recv_cb(struct media_player *plr, int err,
				const struct mpl_cmd_ntf *cmd_ntf)
{
	if (err) {
		LOG_ERR("Command failed (%d)", err);
		return;
	}

	LOG_DBG("Received opcode: %d", cmd_ntf->requested_opcode);

	if (cmd_ntf->requested_opcode == BT_MCS_OPC_PLAY) {
		play_pause_cb(true);
	} else if (cmd_ntf->requested_opcode == BT_MCS_OPC_PAUSE) {
		play_pause_cb(false);
	} else {
		LOG_WRN("Unsupported opcode: %d", cmd_ntf->requested_opcode);
	}
}

/**
 * @brief  Callback handler for getting the current state of the media player.
 *
 *         This callback will be triggered when the server has asked for the
 *         current state of its local media player.
 */
static void mcs_media_state_cb(struct media_player *plr, int err, uint8_t state)
{
	if (err) {
		LOG_ERR("Media state failed (%d)", err);
		return;
	}

	media_player_state = state;
}

/**
 * @brief  Callback handler for getting a pointer to the local media player.
 *
 *         This callback will be triggered during initialization when the
 *         local media player is ready.
 */
static void mcs_local_player_instance_cb(struct media_player *player, int err)
{
	struct mpl_cmd cmd;

	if (err) {
		LOG_ERR("Local player instance failed (%d)", err);
		return;
	}

	LOG_DBG("Received local player");

	local_player = player;

	cmd.opcode = BT_MCS_OPC_PLAY;

	/* Since the media player is default paused when initialized, we
	 * send a play command when the first stream is enabled
	 */
	(void)media_proxy_ctrl_send_command(local_player, &cmd);
}

int ble_mcs_discover(struct bt_conn *conn)
{
	if (!IS_ENABLED(CONFIG_BT_MCC)) {
		LOG_ERR("MCC not enabled");
		return -ECANCELED;
	}

	int ret;
	uint8_t idx = bt_conn_index(conn);

	if (mcp_mcs_disc_status[idx] == FINISHED || mcp_mcs_disc_status[idx] == IN_PROGRESS) {
		return -EALREADY;
	}

	mcp_mcs_disc_status[idx] = IN_PROGRESS;
	ret = bt_mcc_discover_mcs(conn, true);
	if (ret) {
		mcp_mcs_disc_status[idx] = IDLE;
		return ret;
	}

	return 0;
}

int ble_mcs_state_update(struct bt_conn *conn)
{
	if (!IS_ENABLED(CONFIG_BT_MCC)) {
		LOG_ERR("MCC not enabled");
		return -ECANCELED;
	}

	uint8_t idx = bt_conn_index(conn);

	if (mcp_mcs_disc_status[idx] != FINISHED) {
		LOG_WRN("MCS discovery has not finished");
		return -EBUSY;
	}

	return bt_mcc_read_media_state(conn);
}

int ble_mcs_play_pause(struct bt_conn *conn)
{
	int ret = 0;
	struct mpl_cmd cmd;

	if (conn != NULL) {
		uint8_t idx = bt_conn_index(conn);

		if (mcp_mcs_disc_status[idx] != FINISHED) {
			LOG_WRN("MCS discovery has not finished");
			return -EBUSY;
		}
	}

	if (media_player_state == BT_MCS_MEDIA_STATE_PLAYING) {
		cmd.opcode = BT_MCS_OPC_PAUSE;
	} else if (media_player_state == BT_MCS_MEDIA_STATE_PAUSED) {
		cmd.opcode = BT_MCS_OPC_PLAY;
	} else {
		LOG_ERR("Invalid state: %d", media_player_state);
		return -ECANCELED;
	}

	cmd.use_param = false;

	if (IS_ENABLED(CONFIG_BT_MCS)) {
		ret = media_proxy_ctrl_send_command(local_player, &cmd);
	} else if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = bt_mcc_send_cmd(conn, &cmd);
	}

	if (ret) {
		LOG_WRN("Failed to send play/pause command: %d", ret);
		return ret;
	}

	return 0;
}

int ble_mcp_conn_disconnected(struct bt_conn *conn)
{
	if (!IS_ENABLED(CONFIG_BT_MCC)) {
		LOG_ERR("MCC not enabled");
		return -ECANCELED;
	}

	uint8_t idx = bt_conn_index(conn);

	LOG_DBG("MCS discover state reset due to disconnection");
	mcp_mcs_disc_status[idx] = IDLE;
	return 0;
}

int ble_mcs_client_init(void)
{
	if (!IS_ENABLED(CONFIG_BT_MCC)) {
		LOG_ERR("MCC not enabled");
		return -ECANCELED;
	}

	static struct bt_mcc_cb mcc_cb;

	mcc_cb.discover_mcs = mcc_discover_mcs_cb;
	mcc_cb.send_cmd = mcc_send_command_cb;
	mcc_cb.cmd_ntf = mcc_cmd_notification_cb;
	mcc_cb.read_media_state = mcc_read_media_state_cb;
	return bt_mcc_init(&mcc_cb);
}

int ble_mcs_server_init(ble_mcs_play_pause_cb le_audio_play_pause_cb)
{
	if (!IS_ENABLED(CONFIG_BT_MCS)) {
		LOG_ERR("MCS not enabled");
		return -ECANCELED;
	}

	int ret;
	static struct media_proxy_ctrl_cbs mcs_cb;

	play_pause_cb = le_audio_play_pause_cb;

	ret = media_proxy_pl_init();
	if (ret) {
		LOG_ERR("Failed to init media proxy: %d", ret);
		return ret;
	}

	mcs_cb.command_recv = mcs_command_recv_cb;
	mcs_cb.media_state_recv = mcs_media_state_cb;
	mcs_cb.local_player_instance = mcs_local_player_instance_cb;

	ret = media_proxy_ctrl_register(&mcs_cb);
	if (ret) {
		LOG_ERR("Could not init mpl: %d", ret);
		return ret;
	}

	return 0;
}
