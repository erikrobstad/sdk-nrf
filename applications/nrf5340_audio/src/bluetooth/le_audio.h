/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _LE_AUDIO_H_
#define _LE_AUDIO_H_

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <audio_defines.h>

#define LE_AUDIO_SDU_SIZE_OCTETS(bitrate) (bitrate / (1000000 / CONFIG_AUDIO_FRAME_DURATION_US) / 8)

#if (CONFIG_AUDIO_SAMPLE_RATE_48000_HZ)
#define BT_AUDIO_CODEC_CONFIG_FREQ    BT_CODEC_CONFIG_LC3_FREQ_48KHZ
#define BT_AUDIO_CODEC_CAPABILIY_FREQ BT_CODEC_LC3_FREQ_48KHZ
#elif (CONFIG_AUDIO_SAMPLE_RATE_24000_HZ)
#define BT_AUDIO_CODEC_CONFIG_FREQ    BT_CODEC_CONFIG_LC3_FREQ_24KHZ
#define BT_AUDIO_CODEC_CAPABILIY_FREQ BT_CODEC_LC3_FREQ_24KHZ
#elif (CONFIG_AUDIO_SAMPLE_RATE_16000_HZ)
#define BT_AUDIO_CODEC_CONFIG_FREQ    BT_CODEC_CONFIG_LC3_FREQ_16KHZ
#define BT_AUDIO_CODEC_CAPABILIY_FREQ BT_CODEC_LC3_FREQ_16KHZ
#endif /* (CONFIG_AUDIO_SAMPLE_RATE_48000_HZ) */

#define BT_BAP_LC3_PRESET_CONFIGURABLE(_loc, _stream_context, _bitrate)                            \
	BT_BAP_LC3_PRESET(                                                                         \
		BT_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CONFIG_FREQ, BT_CODEC_CONFIG_LC3_DURATION_10,   \
				    _loc, LE_AUDIO_SDU_SIZE_OCTETS(_bitrate), 1, _stream_context), \
		BT_CODEC_LC3_QOS_10_UNFRAMED(LE_AUDIO_SDU_SIZE_OCTETS(_bitrate),                   \
					     CONFIG_BT_AUDIO_RETRANSMITS,                          \
					     CONFIG_BT_AUDIO_MAX_TRANSPORT_LATENCY_MS,             \
					     CONFIG_BT_AUDIO_PRESENTATION_DELAY_US))

#if CONFIG_TRANSPORT_CIS
#if CONFIG_BT_BAP_UNICAST_CONFIGURABLE
#define BT_BAP_LC3_UNICAST_PRESET_NRF5340_AUDIO_SINK                                               \
	BT_BAP_LC3_PRESET_CONFIGURABLE(BT_AUDIO_LOCATION_FRONT_LEFT, BT_AUDIO_CONTEXT_TYPE_MEDIA,  \
				       CONFIG_BT_AUDIO_BITRATE_UNICAST_SINK)

#define BT_BAP_LC3_UNICAST_PRESET_NRF5340_AUDIO_SOURCE                                             \
	BT_BAP_LC3_PRESET_CONFIGURABLE(BT_AUDIO_LOCATION_FRONT_LEFT, BT_AUDIO_CONTEXT_TYPE_MEDIA,  \
				       CONFIG_BT_AUDIO_BITRATE_UNICAST_SRC)

#elif CONFIG_BT_BAP_UNICAST_16_2_1
#define BT_BAP_LC3_UNICAST_PRESET_NRF5340_AUDIO_SINK                                               \
	BT_BAP_LC3_UNICAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT, BT_AUDIO_CONTEXT_TYPE_MEDIA)

#define BT_BAP_LC3_UNICAST_PRESET_NRF5340_AUDIO_SOURCE                                             \
	BT_BAP_LC3_UNICAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT, BT_AUDIO_CONTEXT_TYPE_MEDIA)

#elif CONFIG_BT_BAP_UNICAST_24_2_1
#define BT_BAP_LC3_UNICAST_PRESET_NRF5340_AUDIO_SINK                                               \
	BT_BAP_LC3_UNICAST_PRESET_24_2_1(BT_AUDIO_LOCATION_FRONT_LEFT, BT_AUDIO_CONTEXT_TYPE_MEDIA)

#define BT_BAP_LC3_UNICAST_PRESET_NRF5340_AUDIO_SOURCE                                             \
	BT_BAP_LC3_UNICAST_PRESET_24_2_1(BT_AUDIO_LOCATION_FRONT_LEFT, BT_AUDIO_CONTEXT_TYPE_MEDIA)

#else
#error Unsupported LC3 codec preset for unicast
#endif /* CONFIG_BT_BAP_UNICAST_CONFIGURABLE */
#endif /* CONFIG_TRANSPORT_CIS */

#if CONFIG_TRANSPORT_BIS
#if CONFIG_BT_AUDIO_BROADCAST_CONFIGURABLE
#define BT_BAP_LC3_BROADCAST_PRESET_NRF5340_AUDIO                                                  \
	BT_BAP_LC3_PRESET_CONFIGURABLE(                                                            \
		BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,                      \
		BT_AUDIO_CONTEXT_TYPE_MEDIA, CONFIG_BT_AUDIO_BITRATE_BROADCAST_SRC)

#elif CONFIG_BT_BAP_BROADCAST_16_2_1
#define BT_BAP_LC3_BROADCAST_PRESET_NRF5340_AUDIO                                                  \
	BT_BAP_LC3_BROADCAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT |                          \
						   BT_AUDIO_LOCATION_FRONT_RIGHT,                  \
					   BT_AUDIO_CONTEXT_TYPE_MEDIA)

#elif CONFIG_BT_BAP_BROADCAST_24_2_1
#define BT_BAP_LC3_BROADCAST_PRESET_NRF5340_AUDIO                                                  \
	BT_BAP_LC3_BROADCAST_PRESET_24_2_1(BT_AUDIO_LOCATION_FRONT_LEFT |                          \
						   BT_AUDIO_LOCATION_FRONT_RIGHT,                  \
					   BT_AUDIO_CONTEXT_TYPE_MEDIA)

#elif CONFIG_BT_BAP_BROADCAST_16_2_2
#define BT_BAP_LC3_BROADCAST_PRESET_NRF5340_AUDIO                                                  \
	BT_BAP_LC3_BROADCAST_PRESET_16_2_2(BT_AUDIO_LOCATION_FRONT_LEFT |                          \
						   BT_AUDIO_LOCATION_FRONT_RIGHT,                  \
					   BT_AUDIO_CONTEXT_TYPE_MEDIA)

#elif CONFIG_BT_BAP_BROADCAST_24_2_2
#define BT_BAP_LC3_BROADCAST_PRESET_NRF5340_AUDIO                                                  \
	BT_BAP_LC3_BROADCAST_PRESET_24_2_2(BT_AUDIO_LOCATION_FRONT_LEFT |                          \
						   BT_AUDIO_LOCATION_FRONT_RIGHT,                  \
					   BT_AUDIO_CONTEXT_TYPE_MEDIA)

#else
#error Unsupported LC3 codec preset for broadcast
#endif /* CONFIG_BT_AUDIO_BROADCAST_CONFIGURABLE */
#endif /* CONFIG_TRANSPORT_BIS */

/**
 * @brief Callback for receiving Bluetooth LE Audio data
 *
 * @param	data		Pointer to received data
 * @param	size		Size of received data
 * @param	bad_frame	Indicating if the frame is a bad frame or not
 * @param	sdu_ref		ISO timestamp
 * @param	channel_index	Audio channel index
 */
typedef void (*le_audio_receive_cb)(const uint8_t *const data, size_t size, bool bad_frame,
				    uint32_t sdu_ref, enum audio_channel channel_index,
				    size_t desired_size);

/**
 * @brief	Callback for using the timestamp of the previously sent audio packet
 *
 * @note	Can be used for drift calculation/compensation
 *
 * @param	timestamp	The timestamp
 * @param	adjust		Indicate if the sdu_ref should be used to adjust timing
 */
typedef void (*le_audio_timestamp_cb)(uint32_t timestamp, bool adjust);

/**
 * @brief	Callback for starting to scan for Bluetooth LE advertisements
 */
typedef void (*le_audio_scan_start_cb)(void);

/**
 * @brief	Callback for notifying about nonvalid ISO configs for a connection
 *
 * @note	Nonvalid configs means that the local device does not support the
 *		configurations that the remote device supports
 *
 * @param	conn	The remote device connection
 */
typedef void (*le_audio_nonvalid_iso_cfgs_cb)(struct bt_conn *conn);

enum le_audio_user_defined_action {
	LE_AUDIO_USER_DEFINED_ACTION_1,
	LE_AUDIO_USER_DEFINED_ACTION_2,
	LE_AUDIO_USER_DEFINED_ACTION_NUM
};

/**
 * @brief	Encoded audio data and information.
 * Container for SW codec (typically LC3) compressed audio data.
 */
struct encoded_audio {
	uint8_t const *const data;
	size_t size;
	uint8_t num_ch;
};

/**
 * @brief	Generic function for a user defined button press
 *
 * @param	action	User defined action
 *
 * @return	0 for success,
 *		error otherwise
 */
int le_audio_user_defined_button_press(enum le_audio_user_defined_action action);

/**
 * @brief	Get configuration for audio stream
 *
 * @param	bitrate		Pointer to bitrate used, can be NULL
 * @param	sampling_rate	Pointer to sampling rate used, can be NULL
 * @param	pres_delay	Pointer to presentation delay used, can be NULL
 *
 * @return	0 for success,
 *		-ENXIO if the feature is disabled,
 *		-ENOTSUP if the feature is not supported,
 *		error otherwise
 */
int le_audio_config_get(uint32_t *bitrate, uint32_t *sampling_rate, uint32_t *pres_delay);

/**
 * @brief	Set pointer to connection, used by CIS
 *
 * @param	conn	The connection pointer
 */
void le_audio_conn_set(struct bt_conn *conn);

/**
 * @brief	Set periodic advertising sync
 *
 * @param	pa_sync		Pointer to the periodic advertising sync
 * @param	broadcast_id	Broadcast ID of the periodic advertising
 *
 * @return	0 for success, (negative) error code otherwise
 */
int le_audio_pa_sync_set(struct bt_le_per_adv_sync *pa_sync, uint32_t broadcast_id);

/**
 * @brief	Notify about disconnected connection, used by CIS
 *
 * @param	conn	The connection pointer
 */
void le_audio_conn_disconnected(struct bt_conn *conn);

/**
 * @brief	Set pointer to extended advertisement with periodic adv configured, used by BIS
 *
 * @note	Will also start the broadcast_source
 *
 * @param	ext_adv	Pointer to the extended advertisement
 *
 * @return	0 for success, (negative) error code otherwise
 */
int le_audio_ext_adv_set(struct bt_le_ext_adv *ext_adv);

/**
 * @brief	Get bt_data containing the data to advertise
 *
 * @param	adv		Pointer to the pointer of bt_data to advertise
 * @param	adv_size	Pointer to size of adv
 * @param	periodic	Specify if the data is for periodic advertisement
 */
void le_audio_adv_get(const struct bt_data **adv, size_t *adv_size, bool periodic);

/**
 * @brief	Either resume or pause the Bluetooth LE Audio stream,
 *		depending on the current state of the stream
 *
 * @return	0 for success, error otherwise
 */
int le_audio_play_pause(void);

/**
 * @brief	Send Bluetooth LE Audio data
 *
 * @param	enc_audio	Encoded audio struct
 *
 * @return	0 for success,
 *		-ENXIO if the feature is disabled,
 *		error otherwise
 */
int le_audio_send(struct encoded_audio enc_audio);

/**
 * @brief	Enable Bluetooth LE Audio
 *
 * @param	recv_cb			Callback for receiving Bluetooth LE Audio data
 * @param	timestmp_cb		Callback for using timestamp
 * @param	nonvalid_cfgs_cb	Callback for notifying about nonvalid cfgs for a conn
 *
 * @return	0 for success, error otherwise
 */
int le_audio_enable(le_audio_receive_cb recv_cb, le_audio_timestamp_cb timestmp_cb,
		    le_audio_nonvalid_iso_cfgs_cb nonvalid_cfgs_cb);

/**
 * @brief	Disable Bluetooth LE Audio
 *
 * @return	0 for success, error otherwise
 */
int le_audio_disable(void);

#endif /* _LE_AUDIO_H_ */
