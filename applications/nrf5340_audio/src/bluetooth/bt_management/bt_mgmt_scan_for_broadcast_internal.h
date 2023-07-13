/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_MGMT_SCAN_FOR_BROADCAST_H_
#define _BT_MGMT_SCAN_FOR_BROADCAST_H_

#include <zephyr/bluetooth/bluetooth.h>

/**
 * @brief	Scan for a broadcaster with a given name
 *
 * @param	scan_param	The scan parameters to use
 * @param	name		Broadcast name to search for
 *
 * @return	0 if success, error otherwise
 */
int bt_mgmt_scan_for_broadcast_start(struct bt_le_scan_param *scan_param, char const *const name);

#endif /* _BT_MGMT_SCAN_FOR_BROADCAST_H_ */
