/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BLE_AUDIO_SERVICES_H_
#define _BLE_AUDIO_SERVICES_H_

#include <zephyr/bluetooth/conn.h>

/**
 * @brief  Callback for changing stream state.
 *
 * @param  play Differentiate between play command and pause command.
 */
typedef void (*ble_mcs_play_pause_cb)(bool play);


/**
 * @brief  Discover MCS and included services.
 *
 * @param  conn  Pointer to the active connection, only valid for client.
 *
 * @return 0 for success, error otherwise.
 */
int ble_mcs_discover(struct bt_conn *conn);

/**
 * @brief  Get the current state of the media player, only valid for client.
 *
 * @param  conn  Pointer to the active connection.
 *
 * @return 0 for success, error otherwise.
 */
int ble_mcs_state_update(struct bt_conn *conn);

/**
 * @brief  Send play/pause command to the media player,
 *         depending on the current state.
 *
 * @param  conn  Pointer to the active connection, only valid for client.
 *               Shall be NULL if called from server.
 *
 * @return 0 for success, error otherwise.
 */
int ble_mcs_play_pause(struct bt_conn *conn);

/**
 * @brief  Reset the MCP's MCS discovered state, only valid for client.
 *
 * @param  conn  Pointer to the active connection.
 *
 * @return 0 for success, error otherwise.
 */
int ble_mcp_conn_disconnected(struct bt_conn *conn);


/**
 * @brief  Initialize the Media Control Client.
 *
 * @return 0 for success, error otherwise.
 */
int ble_mcs_client_init(void);

/**
 * @brief  Initialize the Media Control Server.
 *
 * @return 0 for success, error otherwise.
 */
int ble_mcs_server_init(ble_mcs_play_pause_cb le_audio_play_pause_cb);

#endif /* _BLE_AUDIO_SERVICES_H_ */
