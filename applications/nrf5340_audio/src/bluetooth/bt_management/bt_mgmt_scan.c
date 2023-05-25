/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_mgmt_scan.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "ble_hci_vsc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt_scan, CONFIG_BLE_LOG_LEVEL); // TODO: Add BT_MGMT_LOG_LEVEL?

#define CONNECTION_PARAMETERS                                                                      \
	BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL, CONFIG_BLE_ACL_CONN_INTERVAL,               \
			 CONFIG_BLE_ACL_SLAVE_LATENCY, CONFIG_BLE_ACL_SUP_TIMEOUT)

static bt_mgmt_conn_set_cb conn_set;

static uint8_t bonded_num;

static void bond_check(const struct bt_bond_info *info, void *user_data)
{
	char addr_buf[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(&info->addr, addr_buf, BT_ADDR_LE_STR_LEN);

	LOG_DBG("Stored bonding found: %s", addr_buf);
	bonded_num++;
}

static void bond_connect(const struct bt_bond_info *info, void *user_data)
{
	int ret;
	const bt_addr_le_t *adv_addr = user_data;
	struct bt_conn *conn;

	if (!bt_addr_le_cmp(&info->addr, adv_addr)) {
		LOG_DBG("Found bonded device");

		ret = bt_le_scan_stop();
		if (ret) {
			LOG_WRN("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(adv_addr, BT_CONN_LE_CREATE_CONN, CONNECTION_PARAMETERS,
					&conn);
		if (ret) {
			LOG_WRN("Create ACL connection failed: %d", ret);
			bt_mgmt_scan_start();
		}
	}
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];
	uint16_t conn_handle;
	enum ble_hci_vs_tx_power conn_tx_pwr;

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("ACL connection to %s failed, error %d", addr, err);

		bt_conn_unref(conn);
		bt_mgmt_scan_start();

		return;
	}

	/* ACL connection established */
	LOG_INF("Connected: %s", addr);

	ret = bt_hci_get_conn_handle(conn, &conn_handle);
	if (ret) {
		LOG_ERR("Unable to get conn handle");
	} else {
#if (CONFIG_NRF_21540_ACTIVE)
		conn_tx_pwr = CONFIG_NRF_21540_MAIN_DBM;
#else
		conn_tx_pwr = CONFIG_BLE_CONN_TX_POWER_DBM;
#endif /* (CONFIG_NRF_21540_ACTIVE) */
		ret = ble_hci_vsc_conn_tx_pwr_set(conn_handle, conn_tx_pwr);
		if (ret) {
			LOG_ERR("Failed to set TX power for conn");
		} else {
			LOG_DBG("TX power set to %d dBm for connection %p", conn_tx_pwr,
				(void *)conn);
		}
	}

	ret = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (ret) {
		LOG_ERR("Failed to set security to L2: %d", ret);
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	// int ret;
	// uint8_t channel_index;
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	bt_conn_unref(conn);

	// ret = channel_index_get(conn, &channel_index);
	// if (ret) {
	// 	LOG_WRN("Unknown connection");
	// } else {
	// 	disconnected_headset_cleanup(channel_index);
	// }

	bt_mgmt_scan_start();
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	int ret;

	if (err) {
		LOG_ERR("Security failed: level %d err %d", level, err);
		ret = bt_conn_disconnect(conn, err);
		if (ret) {
			LOG_ERR("Failed to disconnect %d", ret);
		}
	} else {
		LOG_DBG("Security changed: level %d", level);
		conn_set(conn);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb,
};

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;

	// if (ble_acl_gateway_all_links_connected()) {
	// 	LOG_DBG("All headset connected");
	// 	return 0;
	// }

	if ((data_len == DEVICE_NAME_PEER_LEN) &&
	    (strncmp(DEVICE_NAME_PEER, data, DEVICE_NAME_PEER_LEN) == 0)) {
		LOG_DBG("Device found");

		ret = bt_le_scan_stop();
		if (ret) {
			LOG_WRN("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, CONNECTION_PARAMETERS, &conn);
		if (ret) {
			LOG_ERR("Could not init connection");
			bt_mgmt_scan_start();
			return ret;
		}

		return 0;
	}

	return -ENOENT;
}

/** @brief  Parse BLE advertisement package.
 */
static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			LOG_WRN("AD malformed");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	switch (type) {
	case BT_GAP_ADV_TYPE_ADV_DIRECT_IND:
		/* Direct advertising has no payload, so no need to parse */
		if (bonded_num) {
			bt_foreach_bond(BT_ID_DEFAULT, bond_connect, (void *)addr);
		}
		break;
	case BT_GAP_ADV_TYPE_ADV_IND:
		/* Fall through */
	case BT_GAP_ADV_TYPE_EXT_ADV:
		/* Fall through */
	case BT_GAP_ADV_TYPE_SCAN_RSP:
		/* Note: May lead to connection creation */
		if (bonded_num < CONFIG_BT_MAX_PAIRED) {
			ad_parse(p_ad, addr);
		}
		break;
	default:
		break;
	}
}

int bt_mgmt_scan_start(void) // TODO: Input interval and window?
{
	int ret;
	/* Scan interval as two times of ISO interval */
	int scan_interval = 10 / 0.625 * 2;
	/* Scan window as two times of ISO interval */
	int scan_window = 10 / 0.625 * 2;
	struct bt_le_scan_param *scan_param =
		BT_LE_SCAN_PARAM(NRF5340_AUDIO_GATEWAY_SCAN_TYPE, BT_LE_SCAN_OPT_FILTER_DUPLICATE,
				 scan_interval, scan_window);

	/* Reset number of bonds found */
	bonded_num = 0;

	bt_foreach_bond(BT_ID_DEFAULT, bond_check, NULL);

	if (bonded_num >= CONFIG_BT_MAX_PAIRED) {
		LOG_INF("All bonded slots filled, will not accept new devices");
	}

	ret = bt_le_scan_start(scan_param, on_device_found);
	if (ret && ret != -EALREADY) {
		LOG_WRN("Scanning failed to start: %d", ret);
		return ret;
	}

	LOG_INF("Scanning successfully started");
	return 0;
}

int bt_mgmt_scan_init(bt_mgmt_conn_set_cb conn_set_cb)
{
	bt_conn_cb_register(&conn_callbacks);

	conn_set = conn_set_cb;

	return 0;
}
