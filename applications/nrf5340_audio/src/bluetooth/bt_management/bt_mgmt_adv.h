/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_MGMT_ADV_H_
#define _BT_MGMT_ADV_H_

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
typedef int (*bt_mgmt_ext_adv_set_cb)(struct bt_le_ext_adv *ext_adv);

/**
 * @brief       Create and start advertising for ACL connection
 *
 * @param[in]	ext_adv		The data to be put in the extended advertisement
 * @param[in]	ext_adv_size	Size of ext_adv
 * @param[in]	per_adv		The data for the periodic advertisement, can be NULL
 * @param[in]	per_adv_size	Size of per_adv
 * @param[in]	connectable	Specify if advertisement should be connectable or not
 *
 * @return	0 if success, error otherwise
 */
int bt_mgmt_adv_start(const struct bt_data *ext_adv, size_t ext_adv_size,
		      const struct bt_data *per_adv, size_t per_adv_size, bool connectable);

/**
 * @brief	Initialize the advertising part of the Bluetooth management module
 *
 * @param[in]	conn_set_cb	The connection set callback
 * @param[in]	ext_adv_set_cb	The extended advertisement set callback, can be NULL
 *
 * @return	0 if success, error otherwise
 */
int bt_mgmt_adv_init(bt_mgmt_conn_set_cb conn_set_cb, bt_mgmt_ext_adv_set_cb ext_adv_set_cb);

#endif /* _BT_MGMT_ADV_H_ */
