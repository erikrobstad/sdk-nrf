/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_CTRL_CFG_H_
#define _BT_CTRL_CFG_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief	Get the Bluetooth controller version from the NET core.
 *
 * @param[out]	ctrl_version	The controller version
 *
 * @return	0 if success, error otherwise
 */
int bt_ctlr_version_get(uint16_t *ctrl_version);

/**
 * @brief	Configure the Bluetooth controller.
 *
 * @param[in]	watchdog_enable	If true, will at given intervals poll the controller
 *				to ensure it is still alive.
 *
 * @return	0 if success, error otherwise
 */
int bt_ctlr_cfg_init(bool watchdog_enable);

#endif
