/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_MGMT_SCAN_H_
#define _BT_MGMT_SCAN_H_

#include "bt_mgmt_adv.h" // TODO: Remove (added because of bt_mgmt_conn_set_cb)

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
 * @brief       Start scanning for advertisements
 *
 * @return      0 if success, error otherwise
 */
int bt_mgmt_scan_start(void);

/**
 * @brief      Initialize the scanning part of the Bluetooth management module
 *
 * @return     0 if success, error otherwise
 */
int bt_mgmt_scan_init(bt_mgmt_conn_set_cb conn_set_cb);

#endif /* _BT_MGMT_SCAN_H_ */
