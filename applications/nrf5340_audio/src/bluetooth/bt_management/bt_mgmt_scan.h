/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_MGMT_SCAN_H_
#define _BT_MGMT_SCAN_H_

#include <zephyr/bluetooth/conn.h>

#include "bt_mgmt_adv.h" // TODO: Remove (added because of bt_mgmt_conn_set_cb and bt_mgmt_conn_disconnected_cb)

#if (CONFIG_SCAN_MODE_ACTIVE)
#define NRF5340_AUDIO_GATEWAY_SCAN_TYPE BT_LE_SCAN_TYPE_ACTIVE
#define NRF5340_AUDIO_GATEWAY_SCAN_PARAMS BT_LE_SCAN_ACTIVE
#elif (CONFIG_SCAN_MODE_PASSIVE)
#define NRF5340_AUDIO_GATEWAY_SCAN_TYPE BT_LE_SCAN_TYPE_PASSIVE
#define NRF5340_AUDIO_GATEWAY_SCAN_PARAMS BT_LE_SCAN_PASSIVE
#else
#error "Please select either CONFIG_SCAN_MODE_ACTIVE or CONFIG_SCAN_MODE_PASSIVE"
#endif

#define DEVICE_NAME_PEER CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_PEER_LEN (sizeof(DEVICE_NAME_PEER) - 1)

/**
 * @brief Disconnect from a remote device or cancel pending connection
 *
 * @param conn   Connection to disconnect
 * @param reason Reason code for the disconnection
 */
void bt_mgmt_conn_disconnect(struct bt_conn *conn, uint8_t reason);

/**
 * @brief Start scanning for advertisements
 */
void bt_mgmt_scan_start(void);

/**
 * @brief Initialize the scanning part of the Bluetooth management module
 *
 * @param conn_set_cb          The connection set callback
 * @param conn_disconnected_cb Callback for notifying about disconnected conn
 * @param scan_interval	       Scan interval in ms
 * @param scan_window	       Scan window in ms
 *
 * @return 0 if success, error otherwise
 */
int bt_mgmt_scan_init(bt_mgmt_conn_set_cb conn_set_cb,
		      bt_mgmt_conn_disconnected_cb conn_disconnected_cb, int scan_interval,
		      int scan_window);

#endif /* _BT_MGMT_SCAN_H_ */
