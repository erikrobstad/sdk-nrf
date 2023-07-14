/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_VOLUME_H_
#define _BT_VOLUME_H_

#include <zephyr/bluetooth/conn.h>

/**
 * @brief	Set volume to a specific value.
 *
 * @param	volume	The absolute volume to set.
 *
 * @return 0 for success,
 *	-ENXIO if the feature is disabled.
 *	Other errors from underlying drivers.
 */
int bt_vol_set(uint8_t volume);

/**
 * @brief	Turn the volume up by one step.
 *
 * @return 0 for success,
 *	-ENXIO if the feature is disabled.
 *	Other errors from underlying drivers.
 */
int bt_vol_up(void);

/**
 * @brief	Turn the volume down by one step.
 *
 * @return 0 for success,
 *	-ENXIO if the feature is disabled.
 *	Other errors from underlying drivers.
 */
int bt_vol_down(void);

/**
 * @brief	Mute the output volume of the device.
 *
 * @return 0 for success,
 *	-ENXIO if the feature is disabled.
 *	Other errors from underlying drivers.
 */
int bt_vol_mute(void);

/**
 * @brief	Unmute the output volume of the device.
 *
 * @return 0 for success,
 *	-ENXIO if the feature is disabled.
 *	Other errors from underlying drivers.
 */
int bt_vol_unmute(void);

/**
 * @brief	Discover VCS and included services.
 *
 * @param	conn	Pointer for peer connection information.
 *
 * @note	This will start a GATT discovery and setup handles and
 *		subscriptions for VCS and included services.
 *		This shall be called once before any other actions related with VCS.
 *
 * @return 0 for success, error otherwise.
 */
int bt_vol_vcs_discover(struct bt_conn *conn);

/**
 * @brief	Initialize the Volume Control Service client.
 *
 * @return 0 for success, error otherwise.
 */
int bt_vol_vcs_ctlr_init(void);

/**
 * @brief	Initialize the Volume Control Service server.
 *
 * @return 0 for success, error otherwise.
 */
int bt_vol_vcs_rend_init(void);

#endif /* _BT_VOLUME_H_ */
