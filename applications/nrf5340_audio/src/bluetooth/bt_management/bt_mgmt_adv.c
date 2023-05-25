/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_mgmt_adv.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "macros_common.h"
#include "ble_hci_vsc.h"
#include "ble_audio_services.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt, CONFIG_BLE_LOG_LEVEL);

#define BT_LE_ADV_FAST_CONN                                                                        \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_1,                      \
			BT_GAP_ADV_FAST_INT_MAX_1, NULL)

static struct k_work adv_work;
static bt_mgmt_conn_set_cb conn_set;
static struct bt_le_ext_adv *adv_ext;
static const struct bt_data *ad_peer_local;
static size_t ad_peer_local_size;

/* Bonded address queue */
K_MSGQ_DEFINE(bonds_queue, sizeof(bt_addr_le_t), CONFIG_BT_MAX_PAIRED, 4);

#if CONFIG_BT_BONDABLE
static void bond_find(const struct bt_bond_info *info, void *user_data)
{
	int ret;
	struct bt_conn *conn;

	/* Filter already connected peers. */
	conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &info->addr);
	if (conn) {
		struct bt_conn_info conn_info;

		ret = bt_conn_get_info(conn, &conn_info);

		if (conn_info.state == BT_CONN_STATE_CONNECTED) {
			LOG_WRN("Already connected");
			bt_conn_unref(conn);
			return;
		}
		bt_conn_unref(conn);
	}

	ret = k_msgq_put(&bonds_queue, (void *)&info->addr, K_NO_WAIT);
	if (ret) {
		LOG_WRN("No space in the queue for the bond");
	}
}
#endif /* CONFIG_BT_BONDABLE */

static void advertising_process(struct k_work *work)
{
	int ret;
	struct bt_le_adv_param adv_param;

#if CONFIG_BT_BONDABLE
	k_msgq_purge(&bonds_queue);
	bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);

	bt_addr_le_t addr;

	if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
		char addr_buf[BT_ADDR_LE_STR_LEN];

		adv_param = *BT_LE_ADV_CONN_DIR_LOW_DUTY(&addr);
		adv_param.id = BT_ID_DEFAULT;
		adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

		/* Clear ADV data set before update to direct advertising */
		ret = bt_le_ext_adv_set_data(adv_ext, NULL, 0, NULL, 0);
		if (ret) {
			LOG_ERR("Failed to clear advertising data. Err: %d", ret);
			return;
		}

		ret = bt_le_ext_adv_update_param(adv_ext, &adv_param);
		if (ret) {
			LOG_ERR("Failed to update ext_adv to direct advertising. Err = %d", ret);
			return;
		}

		bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
		LOG_INF("Set direct advertising to %s", addr_buf);
	} else
#endif /* CONFIG_BT_BONDABLE */
	{
		if (ad_peer_local == NULL) {
			LOG_ERR("Ad_peer not set");
			return;
		}

		ret = bt_le_ext_adv_set_data(adv_ext, ad_peer_local, ad_peer_local_size, NULL, 0);
		if (ret) {
			LOG_ERR("Failed to set advertising data. Err: %d", ret);
			return;
		}
	}

	ret = bt_le_ext_adv_start(adv_ext, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to start advertising set. Err: %d", ret);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];
	uint16_t conn_handle;
	enum ble_hci_vs_tx_power conn_tx_pwr;

	if (err) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
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

	conn_set(bt_conn_ref(conn));
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	bt_conn_unref(conn);
	conn_set(NULL);

	ret = ble_mcp_conn_disconnected(conn);
	if (ret) {
		LOG_ERR("ble_msc_conn_disconnected failed with %d", ret);
	}

	k_work_submit(&adv_work);
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
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb,
};

int bt_mgmt_ext_adv_start(const struct bt_data *ad_peer, size_t adv_size,
			  bt_mgmt_conn_set_cb conn_set_cb)
{
	int ret;

	if (ad_peer == NULL) {
		LOG_ERR("No adv struct receieved");
		return -EINVAL;
	}

	if (adv_size == 0) {
		LOG_ERR("Invalid size of adv struct");
		return -EINVAL;
	}

	conn_set = conn_set_cb;

	ad_peer_local = ad_peer;
	ad_peer_local_size = adv_size;

	bt_conn_cb_register(&conn_callbacks);

	ret = bt_le_ext_adv_create(LE_AUDIO_EXTENDED_ADV_CONN_NAME, NULL, &adv_ext);
	if (ret) {
		LOG_ERR("Failed to create advertising set");
		return ret;
	}

	k_work_submit(&adv_work);

	return 0;
}

int bt_mgmt_init(void)
{
	k_work_init(&adv_work, advertising_process);

	return 0;
}
