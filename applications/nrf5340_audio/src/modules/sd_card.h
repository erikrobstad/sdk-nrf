/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _SD_CARD_H_
#define _SD_CARD_H_

#include <stddef.h>

/**@brief Print out the contents under SD card root path and write the content to buffer.
 *
 * @param[in] path	Path of the folder which going to list
 *			If assigned path is null, then listing the contents under root
 *			If assigned path doesn't exist, an error will be returned
 * @param[in, out] buf Buffer where data is written. If set to NULL, it will be ignored.
 * @param[in] buf_size Buffer size
 * @return	0 on success.
 *              -ENODEV SD init failed. SD card likely not inserted
 *              -EINVAL Failed to append to buffer
 *		Otherwise, error from underlying drivers
 */
int sd_card_list_files(char *path, char *buf, size_t buf_size);

/**@brief Write data from buffer into the file
 *
 * @note If the file already exists, data will be appended to the end of the file.
 *
 * @param[in] filename	Name of the target file for writing, the default location is the
 *			root directoy of SD card, accept absolute path under root of SD card
 * @param[in] data	Data which going to be written into the file
 * @param[in,out] size	Pointer to the number of bytes which is going to be written
 *			The actual written size will be returned
 *
 * @return	0 on success.
 *              -ENODEV SD init failed. SD card likely not inserted
 *		Otherwise, error from underlying drivers
 */
int sd_card_write(char const *const filename, char const *const data, size_t *size);

/**@brief Read data from file into the buffer
 *
 * @param[in] filename	Name of the target file for reading, the default location is the
 *			root directoy of SD card, accept absolute path under root of SD card
 * @param[in] data	The buffer which will be filled by read file contents
 * @param[in,out] size	Pointer to the number of bytes which wait to be read from the file
 *			The actual read size will be returned
 *			If the actual read size is 0, there will be a warning message which
 *			indicates the file is empty
 * @return	0 on success.
 *              -ENODEV SD init failed. SD card likely not inserted
 *		Otherwise, error from underlying drivers
 */
int sd_card_read(char const *const filename, char *const data, size_t *size);

/**@brief  Initialize the SD card interface and print out SD card details.
 *
 * @retval	0 on success
 *              -ENODEV SD init failed. SD card likely not inserted
 *		Otherwise, error from underlying drivers
 */
int sd_card_init(void);

/**@brief   Open file on SD card
 * param[in] filename   Name of file to open
 * param[in] path_to_file Path to file
 * @retval  0 on success
 *              -ENODEV SD init faile. SD likely not inserted
 *      Otherwise, error from undelying drivers
*/
int sd_card_segment_open(char const *const filename, const char *path_to_file);

/**@brief   Read segment on the open file on the SD card
 * param[in] data   Where the read data is stored
 * @param[in,out] size	Number of bytes to be read from file
 *			The actual read size will be returned
 *			If the actual read size is 0, there will be a warning message which
 *			indicates that the file is empty
 * @retval  0 on success
 *              -ENODEV SD init failed. SD likely not inserted
 *      Otherwise, error from undelying drivers
*/
int sd_card_segment_read(char *const data, size_t *size);

/**@brief   Close the file opened by the sd_card_segment_open function
 * @retval  0 on success
 *              -EBUSY Segment read operation has not started
 *      Otherwise, error from undelying drivers
*/

// --------------------------------- TODO: Must update the PR after walktrough
int sd_card_segment_peek(char *const data, size_t *size);
int sd_card_segment_skip(const size_t *size);

int sd_card_segment_close(void);

#endif /* _SD_CARD_H_ */
