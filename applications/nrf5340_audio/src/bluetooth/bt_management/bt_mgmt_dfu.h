/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_MGMT_DFU_H_
#define _BT_MGMT_DFU_H_

/**
 * @brief Enter DFU mode. Advertise SMP_SVR service only
 *
 * @note This call does not return
 */
void bt_mgmt_dfu_start(void);

#endif /* _BT_MGMT_DFU_H_ */
