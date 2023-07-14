/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_REND_H_
#define _BT_REND_H_

#include <zephyr/bluetooth/conn.h>

/**
 * @brief	Adjust volume up by one step
 *
 * @return	0 if success, error otherwise
 */
int bt_rend_volume_up(void);

/**
 * @brief	Adjust volume down by one step
 *
 * @return	0 if success, error otherwise
 */
int bt_rend_volume_down(void);

/**
 * @brief	Set the volume to a given value
 *
 * @param	volume		Value to set the volume to (0-255)
 * @param	from_vcp	Describe if the function was called from a service
 *				or from somewhere else (buttons, shell, etc)
 *
 * @return	0 if success, error otherwise
 */
int bt_rend_volume_set(uint8_t volume, bool from_vcp);

/**
 * @brief	Mute the volume
 *
 * @param	from_vcp	Describe if the function was called from a service
 *				or from somewhere else (buttons, shell, etc)
 *
 * @return	0 if success, error otherwise
 */
int bt_rend_mute(bool from_vcp);

/**
 * @brief	Unmute the volume
 *
 * @return	0 if success, error otherwise
 */
int bt_rend_unmute(void);

/**
 * @brief	Discover rendering services
 *
 * @param	conn	Pointer to the conn to do the discovering on
 *
 * @return	0 if success, error otherwise
 */
int bt_rend_discover(struct bt_conn *conn);

/**
 * @brief	Initialize the rendering services/profiles
 *
 * @return	0 if success, error otherwise
 */
int bt_rend_init(void);

#endif /* _BT_REND_H_ */
