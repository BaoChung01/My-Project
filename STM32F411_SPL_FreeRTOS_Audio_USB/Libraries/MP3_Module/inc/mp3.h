/*
 * File name: mp3.h
 * Content: handle the songs
 */

#ifndef _MP3_H_
#define _MP3_H_

#include "main.h"
#include "core_cm4.h"
#include "stm32f4xx_conf.h"
#include "mp3dec.h"
#include "Audio.h"
#include <string.h>
#include "ff.h"

// Private function prototypes

/*
 * Called by the audio driver when it is time to provide data to
 * one of the audio buffers (while the other buffer is sent to the
 * CODEC using DMA). One mp3 frame is decoded at a time and
 * provided to the audio driver.
 */
void AudioCallback(void *context, int buffer);

/*
 * Taken from
 * http://www.mikrocontroller.net/topic/252319
 */
uint32_t Mp3ReadId3V2Tag(FIL *pInFile, char *pszArtist,
						 uint32_t unArtistSize, char *pszTitle, uint32_t unTitleSize);

/*
 * This function is used to play songs from file name
 * @brief Play song from file name
 * @param filename: filename of the songs
 */
void play_mp3(char *filename);

/*
 * This function is used to play songs from the path and remove the number of files (Seek).
 * @brief Play song from path
 * @param path: Path of folder
 * @param seek: Number of files to ignore
 * @retval FRESULT:
 * FR_OK 		
 * FR_DISK_ERR		
 * FR_INT_ERR	
 * FR_NOT_READY
 * FR_NO_FILE
 * FR_NO_PATH
 * FR_INVALID_NAME
 * FR_DENIED
 * FR_EXIST
 * FR_INVALID_OBJECT
 * FR_WRITE_PROTECTED
 * FR_INVALID_DRIVE
 * FR_NOT_ENABLED
 * FR_NO_FILESYSTEM
 * FR_MKFS_ABORTED
 * FR_TIMEOUT		
*/
FRESULT play_directory(const char *path, unsigned char seek);

#endif /*_MP3_H_*/
