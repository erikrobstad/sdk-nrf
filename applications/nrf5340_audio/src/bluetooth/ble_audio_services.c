/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_audio_services.h"

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/audio/vcs.h>

#include "macros_common.h"
#include "hw_codec.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble_audio_services, CONFIG_LOG_AUDIO_SERVICES_LEVEL);

#define VOLUME_DEFAULT 195
#define VOLUME_STEP 16

static struct bt_vcs *vcs;

#if (CONFIG_BT_VCS_CLIENT)
static struct bt_vcs *vcs_client_peer[CONFIG_BT_MAX_CONN];

static int ble_vcs_client_remote_set(uint8_t channel_num)
{
	if (channel_num > CONFIG_BT_MAX_CONN) {
		return -EPERM;
	}

	if (vcs_client_peer[channel_num] == NULL) {
		return -EINVAL;
	}

	LOG_DBG("VCS client pointed to remote device[%d] %p", channel_num,
		(void *)(vcs_client_peer[channel_num]));
	vcs = vcs_client_peer[channel_num];
	return 0;
}
#endif /* (CONFIG_BT_VCS_CLIENT) */

/**
 * @brief  Convert VCS volume to actual volume setting for HW codec
 *
 *         This range for VCS volume is from 0 to 255 and the
 *         range for HW codec volume is from 0 to 128, this function
 *         converting the VCS volume to HW codec volume setting.
 */
static uint16_t vcs_vol_conversion(uint8_t volume)
{
	return (((uint16_t)volume + 1) / 2);
}

/**
 * @brief  Callback handler for volume state changed.
 *
 *         This callback handler will be triggered if
 *         volume state changed, or muted/unmuted.
 */
static void vcs_state_cb_handler(struct bt_vcs *vcs, int err, uint8_t volume, uint8_t mute)
{
	int ret;

	if (err) {
		LOG_ERR("VCS state callback error: %d", err);
		return;
	}
#if (CONFIG_BT_VCS_CLIENT)
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (vcs == vcs_client_peer[i]) {
			LOG_DBG("VCS state from remote device %d:", i);
		} else {
			ret = ble_vcs_client_remote_set(i);
			/* If remote peer hasn't been connected before,
			 * just skip the operation for it
			 */
			if (ret == -EINVAL) {
				continue;
			}
			ERR_CHK_MSG(ret, "Failed to set VCS client to remote device properly");
			LOG_DBG("Sync with other devices %d", i);
			ret = bt_vcs_vol_set(vcs_client_peer[i], volume);
			if (ret) {
				LOG_DBG("Failed to sync volume to remote device %d, err = %d", i,
					ret);
			}
		}
	}
#endif /* (CONFIG_BT_VCS_CLIENT) */
	LOG_INF("Volume = %d, mute state = %d", volume, mute);
	if (CONFIG_AUDIO_DEV == HEADSET) {
		ret = hw_codec_volume_set(vcs_vol_conversion(volume));
		ERR_CHK_MSG(ret, "Error setting HW codec volume");

		if (mute) {
			ret = hw_codec_volume_mute();
			ERR_CHK_MSG(ret, "Error muting HW codec volume");
		}
	}
}

/**
 * @brief  Callback handler for VCS flags changed.
 *
 *         This callback handler will be triggered if
 *         VCS flags changed.
 */
static void vcs_flags_cb_handler(struct bt_vcs *vcs, int err, uint8_t flags)
{
	if (err) {
		LOG_ERR("VCS flag callback error: %d", err);
	} else {
		LOG_DBG("Volume flags = 0x%01X", flags);
	}
}

int ble_vcs_vol_set(uint8_t volume)
{
#if (CONFIG_BT_VCS_CLIENT)
	int ret;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_vcs_client_remote_set(i);
		/* If remote peer hasn't been connected before, just skip the operation for it */
		if (ret == -EINVAL) {
			continue;
		}
		ERR_CHK_MSG(ret, "Failed to set VCS client to remote device properly");
		ret = bt_vcs_vol_set(vcs, volume);
		if (ret) {
			LOG_WRN("Failed to set volume for remote channel %d, ret = %d", i, ret);
		}
	}
	return 0;
#elif (CONFIG_BT_VCS)
	return bt_vcs_vol_set(vcs, volume);
#endif /* (CONFIG_BT_VCS_CLIENT) */
	return -ENXIO;
}

int ble_vcs_volume_up(void)
{
#if (CONFIG_BT_VCS_CLIENT)
	int ret;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_vcs_client_remote_set(i);
		/* If remote peer hasn't been connected before, just skip the operation for it */
		if (ret == -EINVAL) {
			continue;
		}
		ERR_CHK_MSG(ret, "Failed to set VCS client to remote device properly");
		ret = bt_vcs_unmute_vol_up(vcs);
		if (ret) {
			LOG_WRN("Failed to volume up for remote channel %d, ret = %d", i, ret);
		}
	}
	return 0;
#elif (CONFIG_BT_VCS)
	return bt_vcs_unmute_vol_up(vcs);
#endif /* (CONFIG_BT_VCS_CLIENT) */
	hw_codec_volume_increase();
	return 0;
}

int ble_vcs_volume_down(void)
{
#if (CONFIG_BT_VCS_CLIENT)
	int ret;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_vcs_client_remote_set(i);
		/* If remote peer hasn't been connected before, just skip the operation for it */
		if (ret == -EINVAL) {
			continue;
		}
		ERR_CHK_MSG(ret, "Failed to set VCS client to remote device properly");
		ret = bt_vcs_unmute_vol_down(vcs);
		if (ret) {
			LOG_WRN("Failed to volume down for remote channel %d, ret = %d", i, ret);
		}
	}
	return 0;
#elif (CONFIG_BT_VCS)
	return bt_vcs_unmute_vol_down(vcs);
#endif /* (CONFIG_BT_VCS_CLIENT) */
	hw_codec_volume_decrease();
	return 0;
}

int ble_vcs_volume_mute(void)
{
#if (CONFIG_BT_VCS_CLIENT)
	int ret;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_vcs_client_remote_set(i);
		/* If remote peer hasn't been connected before, just skip the operation for it */
		if (ret == -EINVAL) {
			continue;
		}
		ERR_CHK_MSG(ret, "Failed to set VCS client to remote device properly");
		ret = bt_vcs_mute(vcs);
		if (ret) {
			LOG_WRN("Failed to mute for remote channel %d, ret = %d", i, ret);
		}
	}
	return 0;
#elif (CONFIG_BT_VCS)
	return bt_vcs_mute(vcs);
#endif /* (CONFIG_BT_VCS_CLIENT) */
	return -ENXIO;
}

int ble_vcs_volume_unmute(void)
{
#if (CONFIG_BT_VCS_CLIENT)
	int ret;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_vcs_client_remote_set(i);
		/* If remote peer hasn't been connected before, just skip the operation for it */
		if (ret == -EINVAL) {
			continue;
		}
		ERR_CHK_MSG(ret, "Failed to set VCS client to remote device properly");
		ret = bt_vcs_unmute(vcs);
		if (ret) {
			LOG_WRN("Failed to unmute for remote channel %d, ret = %d", i, ret);
		}
	}
	return 0;
#elif (CONFIG_BT_VCS)
	return bt_vcs_unmute(vcs);
#endif /* (CONFIG_BT_VCS_CLIENT) */
	return -ENXIO;
}

#if (CONFIG_AUDIO_DEV == GATEWAY)
#if (CONFIG_BT_VCS_CLIENT)
/**
 * @brief  Callback handler for VCS discover finished
 *
 *         This callback handler will be triggered when VCS
 *         discover is finished.
 */
static void vcs_dis_cb_handler(struct bt_vcs *vcs, int err, uint8_t vocs_count, uint8_t aics_count)
{
	if (err) {
		LOG_ERR("VCS discover finished callback error: %d", err);
	} else {
		LOG_DBG("VCS discover finished");
	}
}

int ble_vcs_discover(struct bt_conn *conn, uint8_t channel_num)
{
	int ret;

	if (channel_num > CONFIG_BT_MAX_CONN) {
		return -EPERM;
	}
	ret = bt_vcs_discover(conn, &vcs);
	vcs_client_peer[channel_num] = vcs;
	return ret;
}

int ble_vcs_client_init(void)
{
	static struct bt_vcs_cb vcs_client_callback;

	vcs_client_callback.discover = vcs_dis_cb_handler;
	vcs_client_callback.state = vcs_state_cb_handler;
	vcs_client_callback.flags = vcs_flags_cb_handler;
	return bt_vcs_client_cb_register(&vcs_client_callback);
}
#endif /*CONFIG_BT_VCS_CLIENT*/
#endif /*(CONFIG_AUDIO_DEV == GATEWAY)*/

int ble_vcs_server_init(void)
{
	int ret;
	struct bt_vcs_register_param vcs_param;
	static struct bt_vcs_cb vcs_server_callback;

	vcs_server_callback.state = vcs_state_cb_handler;
	vcs_server_callback.flags = vcs_flags_cb_handler;
	vcs_param.cb = &vcs_server_callback;
	vcs_param.mute = BT_VCS_STATE_UNMUTED;
	vcs_param.step = VOLUME_STEP;
	vcs_param.volume = VOLUME_DEFAULT;

	ret = bt_vcs_register(&vcs_param, &vcs);
	if (ret) {
		return ret;
	}

	return 0;
}

#include <zephyr/bluetooth/audio/media_proxy.h>
#include <zephyr/bluetooth/audio/mcc.h>
#include <zephyr/bluetooth/audio/mcs.h>

static void mcc_discover_mcs_cb(struct bt_conn *conn, int err)
{
	LOG_INF("mcc_discover_mcs_cb");

	if (err) {
		LOG_ERR("Discovery of MCS failed (%d)", err);
		return;
	}
}

static void mcc_send_command_cb(struct bt_conn *conn, int err, const struct mpl_cmd *cmd)
{
	LOG_INF("mcc_send_command_cb");

	if (err) {
		LOG_ERR("Command send failed (%d) - opcode: %u, param: %d", err, cmd->opcode,
			cmd->param);
		return;
	}
}

static void mcc_cmd_ntf_cb(struct bt_conn *conn, int err, const struct mpl_cmd_ntf *ntf)
{
	LOG_INF("mcc_cmd_ntf_cb");

	if (err) {
		LOG_ERR("Command notification error (%d) - opcode: %u, result: %u", err,
			ntf->requested_opcode, ntf->result_code);
		return;
	}

	LOG_INF("result_code: %d", ntf->result_code);
}

static struct bt_mcc_cb mcc_cb;

int ble_mcs_client_init(void)
{
	mcc_cb.discover_mcs = mcc_discover_mcs_cb;
	mcc_cb.send_cmd = mcc_send_command_cb;
	mcc_cb.cmd_ntf = mcc_cmd_ntf_cb;

	return bt_mcc_init(&mcc_cb);
}

int ble_mcs_server_init(void)
{
	return media_proxy_pl_init();
}

int ble_mcs_discover(struct bt_conn *conn)
{
	return bt_mcc_discover_mcs(conn, true);
}

int ble_mcs_play(struct bt_conn *conn)
{
	int ret;
	struct mpl_cmd cmd;

	cmd.opcode = BT_MCS_OPC_PLAY;
	cmd.use_param = false;

	ret = bt_mcc_send_cmd(conn, &cmd);
	if (ret) {
		LOG_ERR("Failed to send play command: %d", ret);
		return ret;
	}

	return 0;
}

int ble_mcs_pause(struct bt_conn *conn)
{
	int ret;
	struct mpl_cmd cmd;

	cmd.opcode = BT_MCS_OPC_PAUSE;
	cmd.use_param = false;

	ret = bt_mcc_send_cmd(conn, &cmd);
	if (ret) {
		LOG_ERR("Failed to send pause command: %d", ret);
		return ret;
	}

	return 0;
}
