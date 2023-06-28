/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_mgmt.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "macros_common.h"
#include "nrf5340_audio_common.h"
#include "ble_audio_services.h"
#include "ble_hci_vsc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt, CONFIG_BT_MGMT_LOG_LEVEL);

ZBUS_CHAN_DEFINE(bt_mgmt_chan, struct bt_mgmt_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

/**
 * @brief	Iterative function used to find connected conns
 *
 * @param	conn	The connection to check
 * @param	data	Pointer to store number of valid conns
 */
static void conn_state_connected_check(struct bt_conn *conn, void *data)
{
	int ret;
	uint8_t *num_conn = (uint8_t *)data;
	struct bt_conn_info info;

	ret = bt_conn_get_info(conn, &info);
	if (ret) {
		LOG_ERR("Failed to get conn info for %p: %d", (void *)conn, ret);
		return;
	}

	if (info.state != BT_CONN_STATE_CONNECTED) {
		return;
	}

	(*num_conn)++;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];
	uint8_t num_conn;
	uint16_t conn_handle;
	enum ble_hci_vs_tx_power conn_tx_pwr;
	struct bt_mgmt_msg msg;

	if (err) {
		LOG_ERR("ACL connection to %s failed, error %d", addr, err);

		bt_conn_unref(conn);

		if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
			bt_mgmt_scan_start(0, 0);
		}

		if (IS_ENABLED(CONFIG_BT_PERIPHERAL)) {
			/* TODO: Start adv? */
		}

		return;
	}

	bt_conn_foreach(BT_CONN_TYPE_LE, conn_state_connected_check, (void *)&num_conn);

	/* ACL connection established */
	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected: %s", addr);

#if defined(CONFIG_BT_MAX_CONN) // TODO: What if CONFIG_BT_MAX_CONN is not enabled?
	if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
		if (num_conn < CONFIG_BT_MAX_CONN) {
			bt_mgmt_scan_start(0, 0);
		}
	}
#endif /* (CONFIG_BT_MAX_CONN) */

	ret = bt_hci_get_conn_handle(conn, &conn_handle);
	if (ret) {
		LOG_ERR("Unable to get conn handle");
	} else {
		if (IS_ENABLED(CONFIG_NRF_21540_ACTIVE)) {
			conn_tx_pwr = CONFIG_NRF_21540_MAIN_DBM;
		} else {
			conn_tx_pwr = CONFIG_BLE_CONN_TX_POWER_DBM;
		}

		ret = ble_hci_vsc_conn_tx_pwr_set(conn_handle, conn_tx_pwr);
		if (ret) {
			LOG_ERR("Failed to set TX power for conn");
		} else {
			LOG_DBG("TX power set to %d dBm for connection %p", conn_tx_pwr,
				(void *)conn);
		}
	}

	msg.event = BT_MGMT_CONNECTED;
	msg.conn = conn;

	ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);

	if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
		ret = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (ret) {
			LOG_ERR("Failed to set security to L2: %d", ret);
		}
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_mgmt_msg msg;

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
		bt_conn_unref(conn);
	}

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = ble_mcp_conn_disconnected(conn);
		if (ret) {
			LOG_ERR("ble_msc_conn_disconnected failed with %d", ret);
		}
	}

	/* Publish disconnected */
	msg.event = BT_MGMT_DISCONNECTED;
	ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);

	if (IS_ENABLED(CONFIG_BT_PERIPHERAL)) {
		ret = bt_mgmt_adv_restart();
		ERR_CHK(ret);
	}

	if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
		bt_mgmt_scan_start(0, 0);
	}
}

#if defined(CONFIG_BT_SMP)
static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	int ret;
	struct bt_mgmt_msg msg;

	if (err) {
		LOG_ERR("Security failed: level %d err %d", level, err);
		ret = bt_conn_disconnect(conn, err);
		if (ret) {
			LOG_ERR("Failed to disconnect %d", ret);
		}
	} else {
		LOG_DBG("Security changed: level %d", level);
		/* Publish connected */
		msg.event = BT_MGMT_SECURITY_CHANGED;
		msg.conn = conn;

		ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
		ERR_CHK(ret);
	}
}
#endif /* defined(CONFIG_BT_SMP) */

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
#if defined(CONFIG_BT_SMP)
	.security_changed = security_changed_cb,
#endif /* defined(CONFIG_BT_SMP) */
};

void bt_mgmt_conn_disconnect(struct bt_conn *conn, uint8_t reason)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_CONN)) {
		ret = bt_conn_disconnect(conn, reason);
		if (ret) {
			LOG_ERR("Failed to disconnect connection %p (%d)", (void *)conn, ret);
		}
	} // TODO: else print warning?
}

int bt_mgmt_init(void)
{
	if (IS_ENABLED(CONFIG_BT_CONN)) {
		bt_conn_cb_register(&conn_callbacks);
	}

	if (IS_ENABLED(CONFIG_BT_PERIPHERAL) || IS_ENABLED(CONFIG_BT_BROADCASTER)) {
		bt_mgmt_adv_init();
	}

	return 0;
}
