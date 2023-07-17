#ifndef _AUDIO_LC3_H_
#define _AUDIO_LC3_H_
#include <stdint.h>



int audio_lc3_play(const char *filename, const char *path_to_file);
int audio_lc3_sw_codec_decoder_init(uint16_t frame_duration_ms, uint32_t sample_rate, uint8_t bit_depth, uint8_t channels);
void audio_lc3_i2s_callback(uint32_t frame_start_ts, uint32_t *rx_buf_released, uint32_t const *tx_buf_released);
int audio_lc3_play_init(const char *filename, const char *path_to_file);

void audio_lc3_buffers_set(uint8_t *tx_buf);

#endif