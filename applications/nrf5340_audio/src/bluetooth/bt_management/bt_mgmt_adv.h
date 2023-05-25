/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_MGMT_H_
#define _BT_MGMT_H_

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#define LE_AUDIO_EXTENDED_ADV_NAME                                                                 \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_NAME,                            \
			CONFIG_BLE_ACL_EXT_ADV_INT_MIN, CONFIG_BLE_ACL_EXT_ADV_INT_MAX, NULL)

#define LE_AUDIO_EXTENDED_ADV_CONN_NAME                                                            \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONNECTABLE |                        \
				BT_LE_ADV_OPT_USE_NAME,                                            \
			CONFIG_BLE_ACL_EXT_ADV_INT_MIN, CONFIG_BLE_ACL_EXT_ADV_INT_MAX, NULL)

#define LE_AUDIO_PERIODIC_ADV                                                                      \
	BT_LE_PER_ADV_PARAM(CONFIG_BLE_ACL_PER_ADV_INT_MIN, CONFIG_BLE_ACL_PER_ADV_INT_MAX,        \
			    BT_LE_PER_ADV_OPT_NONE)

typedef void (*bt_mgmt_conn_set_cb)(struct bt_conn *conn);

/**
 * @brief       Create and start extended advertising for ACL connection
 *
 * @param[in]   ad_peer      The data to be put in the extended advertisment
 * @param[in]   adv_size     Size of ad_peer
 * @param[in]   conn_set_cb  The connection set callback
 *
 * @return      0 if success, error otherwise
 */
int bt_mgmt_ext_adv_start(const struct bt_data *ad_peer, size_t adv_size,
			  bt_mgmt_conn_set_cb conn_set_cb);

/**
 * @brief      Initialize the Bluetooth management module
 *
 * @return     0 if success, error otherwise
 */
int bt_mgmt_init(void);

#endif /* _BT_MGMT_H_ */
