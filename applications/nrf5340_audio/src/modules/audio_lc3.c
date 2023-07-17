#include "audio_lc3.h"
#include <stdint.h>
#include <zephyr/sys/ring_buffer.h>
#include "sd_card.h"
#include "hw_codec.h"
#include "sw_codec_lc3.h"
#include "pcm_stream_channel_modifier.h"
#include "audio_i2s.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_lc3, CONFIG_MODULE_AUDIO_LC3_LOG_LEVEL);

#define AUDIO_LC3_DEFAULT_VOLUME 50
// This is not really how I want to do it. Would rather get the bit depth info from header in file or something.
#define AUDIO_LC3_BIT_DEPTH 16
/* Amount of 16-bit samples that fit within a block */
#define I2S_16BIT_SAMPLES_NUM (I2S_SAMPLES_NUM * 2)
#define I2S_8BIT_SAMPLES_NUM (I2S_16BIT_SAMPLES_NUM * 2)

/*File structure of the LC3 encoded files*/
typedef struct {
	uint16_t fileId; /* Constant value, 0xCC1C */
	uint16_t hdrSize; /* Header size, 0x0012 */
	uint16_t sampleRate_divided100; /* Sample frequency / 100 */
	uint16_t bitRate_divided100; /* Bit rate / 100 (total for all channels) */
	uint16_t channels; /* Number of channels */
	uint16_t frameMs_times100; /* Frame duration in ms * 100 */
	uint16_t rfu; /* RFU value */
	uint16_t signalLen; /* Number of samples in signal, 16 LSB */
	uint16_t signalLenRed; /* Number of samples in signal, 16 MSB (>> 16) */
} lc3BinaryHdr_t;

static lc3BinaryHdr_t header;
static size_t header_size = sizeof(lc3BinaryHdr_t);
static uint16_t lc3_frame_length;
static uint16_t lc3_frames_num;
static uint16_t pcm_mono_frame_size;
static uint16_t pcm_stereo_frame_size;
static uint16_t ringbuf_size;
static uint8_t bit_depth;

static bool audio_lc3_module_initialized;

static uint16_t m_i2s_tx_buf_a[I2S_16BIT_SAMPLES_NUM], m_i2s_rx_buf_a[I2S_16BIT_SAMPLES_NUM], m_i2s_tx_buf_b[I2S_16BIT_SAMPLES_NUM], m_i2s_rx_buf_b[I2S_16BIT_SAMPLES_NUM];

RING_BUF_DECLARE(m_ringbuf_sound_data_lc3, 3840);
K_SEM_DEFINE(m_sem_load_from_buf_lc3, 0, 1);

void audio_lc3_buffers_set(uint8_t *tx_buf)
{
	if(ring_buf_space_get(&m_ringbuf_sound_data_lc3) > pcm_stereo_frame_size) {
		k_sem_give(&m_sem_load_from_buf_lc3);
	}
}

// ----------------------- Must remove this
void audio_lc3_i2s_callback(uint32_t frame_start_ts, uint32_t *rx_buf_released, uint32_t const *tx_buf_released)
{
	// Update the I2S buffers by reading from the ringbuffer
	if((uint16_t *)tx_buf_released == m_i2s_tx_buf_a) {
		ring_buf_get(&m_ringbuf_sound_data_lc3, (uint8_t *)m_i2s_tx_buf_a, I2S_8BIT_SAMPLES_NUM);
		audio_i2s_set_next_buf((const uint8_t *)m_i2s_tx_buf_a, (uint32_t *)m_i2s_rx_buf_a);
	} else if((uint16_t *)tx_buf_released == m_i2s_tx_buf_b) {
		ring_buf_get(&m_ringbuf_sound_data_lc3, (uint8_t *)m_i2s_tx_buf_b, I2S_8BIT_SAMPLES_NUM);
		audio_i2s_set_next_buf((const uint8_t *)m_i2s_tx_buf_b, (uint32_t *)m_i2s_rx_buf_b);
	} else {
		LOG_ERR("Should not happen! 0x%x\n", (int)tx_buf_released);
		return;
	}

	// Check the current free space in the buffer.
	// If more than half the buffer is free we should move more data from the SD card
	if(ring_buf_space_get(&m_ringbuf_sound_data_lc3) > pcm_stereo_frame_size) {
		k_sem_give(&m_sem_load_from_buf_lc3);
	}
}

int audio_lc3_buffer_to_ringbuffer(uint8_t *buffer, size_t numbytes){
	/* Burde kanskje ha en sjekk her for å sjekke at numbytes gir mening */
	static uint8_t *buf_ptr;

	numbytes = ring_buf_put_claim(&m_ringbuf_sound_data_lc3, &buf_ptr, numbytes);

	for (int i = 0; i<numbytes; i++){
		buf_ptr[i] = buffer[i];
	}
	ring_buf_put_finish(&m_ringbuf_sound_data_lc3, numbytes);
	return numbytes;
}

int audio_lc3_play(const char *filename, const char *path_to_file)
{
	int ret;

	if (!audio_lc3_module_initialized){
		LOG_ERR("Audio lc3 module is uninitialized");
		return -1; // Have to find a fitting return value here
	}

	uint16_t pcm_mono_frame[pcm_mono_frame_size / 2];
	uint16_t pcm_stereo_frame[pcm_stereo_frame_size / 2];
	uint16_t pcm_mono_write_size;
	size_t pcm_stereo_write_size;
	uint16_t lc3_frame[lc3_frame_length];
	size_t lc3_fr_len = lc3_frame_length;
	size_t lc3_fr_len_size = sizeof(lc3_frame_length);

	/* First, open file on SD card */
	ret = sd_card_segment_open(filename, path_to_file);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}

	/* Then, skip the header */
	ret = sd_card_segment_skip(&header_size);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}

	ret = audio_lc3_sw_codec_decoder_init(header.frameMs_times100 / 100, // TODO: Have this in an init function?
					(uint32_t)(header.sampleRate_divided100 * 100),
					bit_depth, (char)header.channels);
	if (ret < 0){
		LOG_ERR("Error when initializing audio sw codec\n");
		return ret;
	}

	ret = audio_lc3_set_up_i2s_transmission();
	if (ret < 0){
		LOG_ERR("Error when setting up audio i2s transmission. Return value: %d", ret);
		return ret;
	}

	ret = hw_codec_volume_set(AUDIO_LC3_DEFAULT_VOLUME);
	if (ret < 0){
		LOG_ERR("Error setting volume on hw codec. Return value: %d", ret);
		return ret;
	}
	LOG_DBG("Starting to play audio.");
	for (uint32_t i = 0; i < lc3_frames_num; i++) {
		/* Skip the frame length info to get to the audio data */
		ret = sd_card_segment_skip(&lc3_fr_len_size);
		if (ret < 0) {
			LOG_ERR("Error when trying to skip file on SD card. Return value: %d", ret);
			return ret;
		}

		/* Read the audio data frame to be encoded */
		ret = sd_card_segment_read((char *)lc3_frame, &lc3_fr_len);
		if(ret < 0){
			LOG_ERR("Something went wrong when reading from SD card. Return value: %d", ret);
			return ret;
		}

		/* Decode audio data frame*/
		// Should maybe figure out a better way of passing parameter number 4 here...
		ret = sw_codec_lc3_dec_run((char *)lc3_frame, lc3_frame_length,
					   pcm_stereo_frame_size, 0, pcm_mono_frame,
					   &pcm_mono_write_size, false);
		if (ret < 0) {
			LOG_ERR("Error when running decoder. Return value: %d\n", ret);
			return ret;
		}
		/* Convert from mono to stereo */
		ret = pscm_zero_pad(pcm_mono_frame, pcm_mono_write_size, 0, bit_depth, pcm_stereo_frame, &pcm_stereo_write_size);
		if (ret < 0) {
			LOG_ERR("Error when converting to stereo. Return value: %d", ret);
			return ret;
		}
		/* Wait until there is enough space in the ringbuffer */
		k_sem_take(&m_sem_load_from_buf_lc3, K_FOREVER);
		audio_lc3_buffer_to_ringbuffer((char *)pcm_stereo_frame, pcm_stereo_write_size);
	}

	ret = hw_codec_volume_mute();
	if (ret < 0){
		LOG_ERR("Error muting volume on hw codec. Return value: %d", ret);
		return ret;
	}

	LOG_DBG("Done playing audio");
	ret = sd_card_segment_close();
	if (ret < 0){
		LOG_ERR("Error when closing file. Return value: %d", ret);
		return ret;
	}
	return 0;
}

// ----------------------- Must remove this
int audio_lc3_set_up_i2s_transmission(){
	/* Start I2S transmission by setting up both buffersets */
	// Unsure whether I should include the commented lines or not. Looks cleaner without them, but maybe it is a little wrong.
	// ring_buf_get(&m_ringbuf_sound_data_lc3, (uint8_t *)m_i2s_tx_buf_a, I2S_8BIT_SAMPLES_NUM);
	audio_i2s_start((const uint8_t *)m_i2s_tx_buf_a, (uint32_t *)m_i2s_rx_buf_a);
	// ring_buf_get(&m_ringbuf_sound_data_lc3, (uint8_t *)m_i2s_tx_buf_b, I2S_8BIT_SAMPLES_NUM);
	audio_i2s_set_next_buf((const uint8_t *)m_i2s_tx_buf_b, (uint32_t *)m_i2s_rx_buf_b);
	return 0;
}

int audio_lc3_sw_codec_decoder_init(uint16_t frame_duration_ms, uint32_t sample_rate,
				    uint8_t bit_depth, uint8_t channels)
{
	int ret;
	uint16_t frame_duration_us = frame_duration_ms * 1000;

	ret = sw_codec_lc3_init(NULL, NULL, frame_duration_us);
	if (ret < 0) {
		LOG_ERR("Sw codec failed to initialize. Return value: %d", ret);
		return ret;
	}
	LOG_DBG("Sw codec initialized successfully.");


	ret = sw_codec_lc3_dec_init(sample_rate, bit_depth, frame_duration_us, channels);
	if (ret < 0) {
		LOG_ERR("Sw codec decoder failed to initialize. Return value: %d\n", ret);
		return ret;
	}
	LOG_DBG("Sw codec decoder initialized successfully.");

	return 0;
}


// ----------------------- Get LC3 info from the specific file - Only need the audio_lc3_play() function, and not this one?
int audio_lc3_play_init(const char *filename, const char *path_to_file){
	int ret;
	size_t lc3_frame_length_size = sizeof(lc3_frame_length);
	uint32_t num_samples;

	encoder_thread_id =
			k_thread_create(&encoder_thread_data, encoder_thread_stack,
					CONFIG_ENCODER_STACK_SIZE, (k_thread_entry_t)encoder_thread,
					NULL, NULL, NULL,
					K_PRIO_PREEMPT(CONFIG_ENCODER_THREAD_PRIO), 0, K_NO_WAIT);
		ret = k_thread_name_set(encoder_thread_id, "ENCODER");
		ERR_CHK(ret);

	ret = sd_card_segment_open(filename, path_to_file);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}

	/* Read the header */
	ret = sd_card_segment_read((char *)&header, &header_size);
	if (ret < 0) {
		LOG_ERR("Error when trying to peek at file segment on SD card. Return value: %d", ret);
		return ret;
	}

	// remember that this is in bytes and not 16-bit samples
	pcm_mono_frame_size = 2 * header.sampleRate_divided100 * header.frameMs_times100 / 1000;
	pcm_stereo_frame_size = pcm_mono_frame_size * 2;
	ringbuf_size = pcm_stereo_frame_size * 2;
	num_samples = (header.signalLenRed << 16) + header.signalLen;
	lc3_frames_num = 2 * num_samples / pcm_mono_frame_size;
	bit_depth = AUDIO_LC3_BIT_DEPTH;
	
	/* Read the frame length */
	ret = sd_card_segment_peek((char *)&lc3_frame_length, &lc3_frame_length_size);
	if (ret < 0){
		LOG_ERR("Error when trying to peek at file  segment on SD card. Return value: %d", ret);
		return ret;
	}

	ret = sd_card_segment_close();
	if (ret < 0) {
		LOG_ERR("Error when trying to close file on SD card. Return value: %d", ret);
		return ret;
	}

	audio_lc3_module_initialized = true;
	return 0;
}