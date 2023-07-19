#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library_include_directories(
    content
)

target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/bt_content_control.c)

if (CONFIG_BT_MCC OR CONFIG_BT_MCS)
target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/content/bt_media_control.c)
endif()
