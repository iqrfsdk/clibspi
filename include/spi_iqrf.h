/*
 * Copyright 2015 MICRORISC s.r.o.
 * Copyright 2018 IQRF Tech s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * spi_iqrf library serves as an programming interface for communication
 * between Linux system and TR modules using SPI and IO. IO functionality is build
 * up on 'gpio' static library.
 *
 * @file		spi_iqrf.h
 * @date		11.3.2018
 */

#ifndef __SPI_IQRF_H
#define __SPI_IQRF_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "errors.h"
#include "declspec.h"

//#ifdef SPI_IQRF_EXPORTS
//#define SPI_IQRF_DECLSPEC LIB_EXPORT_SHARED
//#else
//#define SPI_IQRF_DECLSPEC LIB_IMPORT_SHARED
//#endif

#define SPI_IQRF_DECLSPEC

/** SPI device name size */
// Constant is used to specify array. Therefore, they can not be defined as static const uint8_t.
#define SPI_DEV_CAPACITY 128

/** IQRF SPI Error constants. */
typedef enum spi_iqrf_Errors {
	///< An enum constant representing results without errors
	BASE_TYPES_OPER_OK = 0,
	///< An enum constant representing results where some error occures
	BASE_TYPES_OPER_ERROR = -1,
	///< An enum constant representing operation on not initialized library
	BASE_TYPES_LIB_NOT_INITIALIZED = -2,
	///< An enum constant representing bad gpio_getValue of SPI Status was returned from SPI device
	SPI_IQRF_ERROR_BAD_STATUS = -10,
	///< An enum constant representing the spi iqrf error CRCS mismatch
	SPI_IQRF_ERROR_CRCS = -11 /**< CRCS mismatch */
} spi_iqrf_Errors;

/**
 * Values according to the table in IQRF SPI User's guide (chapter SPI status). Status, which is
 * different from Data Ready SPI status.
 */
typedef enum spi_iqrf_SPIStatus_DataNotReady {
	///< An enum constant representing thah SPI is not active (disabled by the disableSPI() command)
	SPI_IQRF_SPI_DISABLED = 0x0,
	///< An enum constant representing that SPI is suspended by the stopSPI() command
	SPI_IQRF_SPI_SUSPENDED = 0x07,
	///< An enum constant representing that SPI is not ready (buffer full, last CRCM O.K.).
	///	Data in bufferCOM is protected against overwriting by next transmission from the master.
	SPI_IQRF_SPI_BUFF_PROTECT = 0x3F,
	///< An enum constant representing that SPI is not ready (buffer full, last CRCM error).
	SPI_IQRF_SPI_CRCM_ERR = 0x3E,
	///< An enum constant representing that SPI is ready � communication mode
	SPI_IQRF_SPI_READY_COMM = 0x80,
	///< An enum constant representing that SPI is ready � programming mode
	SPI_IQRF_SPI_READY_PROG = 0x81,
	///< An enum constant representing that SPI is ready � debugging mode
	SPI_IQRF_SPI_READY_DEBUG = 0x82,
	///< An enum constant representing that SPI is probably in slow communication mode
	// TODO: Check the right meaning.
	SPI_IQRF_SPI_SLOW_MODE = 0x83,
	///< An enum constant representing that SPI not active (HW error)
	SPI_IQRF_SPI_HW_ERROR = 0xFF
} spi_iqrf_SPIStatus_DataNotReady;

/**
 * Current SPI status.
 */
typedef struct spi_iqrf_SPIStatus {
	/** determines if dataReady field is valid. */
	int isDataReady;

	union {
		/** Current status of SPI in case, that data is not ready. */
		spi_iqrf_SPIStatus_DataNotReady dataNotReadyStatus;
		/** Count of available data in case of data redady response.*/
		int dataReady;
	};
} spi_iqrf_SPIStatus;

/**
 * Other common constants.
 */
typedef enum spi_iqrf_CommonConstants {
	///< An enum consant representing maximal length of data to be write or read to or from SPI slave.
	SPI_IQRF_MAX_DATA_LENGTH = 128
} spi_iqrf_CommonConstants;

/**
 * Defines an alias representing IQRF communication modes.
 *
 * @remark Communication mode depends on used IQRF module.
 * @remark Default communication mode is low speed for 5x modules.
 */
typedef enum _spi_iqrf_CommunicationMode
{
	///< An enum constant representing the low speed communication mode option
	SPI_IQRF_LOW_SPEED_MODE,
	///< An enum constant representing the high speed comunication mode option
	SPI_IQRF_HIGH_SPEED_MODE
} spi_iqrf_CommunicationMode;

typedef struct spi_iqrf_config_struct
{
  /** Device file name*/
  char spiDev[SPI_DEV_CAPACITY+1];
  uint8_t enableGpioPin;
  uint8_t spiMasterEnGpioPin;
  uint8_t spiPgmSwGpioPin;
} spi_iqrf_config_struct;

/** Default SPI device. */
#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
	#define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev0.0"
#endif //SPI_IQRF_DEFAULT_SPI_DEVICE

/** Programming targets */
// Constants are used in case. Therefore, they can not be defined as static const uint8_t.
#define CFG_TARGET              0x00
#define RFPMG_TARGET            0x01
#define RFBAND_TARGET           0x02
#define ACCESS_PWD_TARGET       0x03
#define USER_KEY_TARGET         0x04
#define FLASH_TARGET            0x05
#define INTERNAL_EEPROM_TARGET  0x06
#define EXTERNAL_EEPROM_TARGET  0x07
#define SPECIAL_TARGET          0x08


/**
 * Initializes SPI device to use.
 *
 * @param	dev	SPI device to communicate with.
 *
 * @return	@c BASE_TYPES_OPER_OK if initialization has performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR if an error has occurred during initialization or if library
 * 			is already initialized.
 */
SPI_IQRF_DECLSPEC int spi_iqrf_init(const char *dev);

/**
 * Initializes default SPI device to use. SPI device to communicate with will be the @c
 * SPI_IQRF_DEFAULT_SPI_DEVICE.
 *
 * @return	@c BASE_TYPES_OPER_OK if initialization has performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR if an error has occurred during initialization or if library
 * 			is already initialized.
 */
SPI_IQRF_DECLSPEC int spi_iqrf_initDefault(void);

/**
* Initialization of the SPI for IQRF module with advanced setting
*
* @param	configStruct - advaced configuration structure
*
* @return	@c BASE_TYPES_OPER_ERROR = initialization failed
* @return	@c BASE_TYPES_OPER_OK = initialization was correct
*/
SPI_IQRF_DECLSPEC int spi_iqrf_initAdvanced(const spi_iqrf_config_struct *configStruct);

/**
 * Get actual communication mode of IQRF SPI library.
 *
 * @return	Actual communication mode of IQRF SPI library.
 */
SPI_IQRF_DECLSPEC spi_iqrf_CommunicationMode spi_iqrf_getCommunicationMode(void);

/**
 * Set SPI IQRF communication mode of IQRF SPI library.
 *
 * @param	communicationMode	Communication mode.
 *
 * @return	@c BASE_TYPES_OPER_OK if communication mode was changed.
 * @return	@c BASE_TYPES_LIB_NOT_INITIALIZED  if the library has not been initialized.
 * @return	@c BASE_TYPES_OPER_ERROR if an undefined communication mode is in parameter @c
 * 			communicationMode.
 */
SPI_IQRF_DECLSPEC int spi_iqrf_setCommunicationMode(spi_iqrf_CommunicationMode communication_mode);

/**
 * Returns SPI status of TR module.
 *
 * @param [in,out]	spiStatus	returned SPI status, cannot be @c NULL.
 *
 * @return	@c BASE_TYPES_OPER_OK if operation has performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR if an error has occurred during operation. This includes the
 * 			situation when @c spiStatus has had @c NULL gpio_getValue.
 * @return	@c SPI_IQRF_ERROR_BAD_STATUS if returned status gpio_getValue is incorrect.
 * @return	@c BASE_TYPES_LIB_NOT_INITIALIZED if the library has not been initialized.
 */
SPI_IQRF_DECLSPEC int spi_iqrf_getSPIStatus(spi_iqrf_SPIStatus *spiStatus);

/**
 * Writes specified data into the module.
 *
 * @param [in,out]	dataToWrite	data to write, cannot be @c NULL.
 * @param	dataLen			   	length (in bytes) of data to write, valid gpio_getValue must be in
 * 								interval of (0, SPI_IQRF_MAX_DATA_LENGTH>
 *
 * @return	@c BASE_TYPES_OPER_OK if operation has performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR if an error has occurred during operation.
 * @return	@c BASE_TYPES_LIB_NOT_INITIALIZED if the library has not been initialized.
 */
SPI_IQRF_DECLSPEC int spi_iqrf_write(void *dataToWrite, unsigned int dataLen);

/**
 * Read specified number of bytes from the module and stores them into specified buffer.
 *
 * @param [in,out]	readBuffer	buffer to store data from module, cannot be @c NULL.
 * @param	dataLen			  	length (in bytes) of data to read, valid gpio_getValue must be in
 * 								interval of (0, SPI_IQRF_MAX_DATA_LENGTH>
 *
 * @return	@c BASE_TYPES_OPER_OK if operation has performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR if an error has occurred during operation.
 * @return	@c SPI_IQRF_ERROR_CRCS if CRSC of returned data doesn't match.
 * @return	@c BASE_TYPES_LIB_NOT_INITIALIZED if the library has not been initialized.
 */
SPI_IQRF_DECLSPEC int spi_iqrf_read(void *readBuffer, unsigned int dataLen);

/**
* Reads TR Module Info from SPI
*
* @param	readBuffer	- TR module info is read to this buffer
* @param	dataLen		- length (in bytes) of the data, at least 16B
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during read operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c SPI_IQRF_ERROR_CRCS = mismatched CRC
* @return	@c BASE_TYPES_OPER_OK = data were successfully read
*/
SPI_IQRF_DECLSPEC int spi_iqrf_get_tr_module_info(void *readBuffer, unsigned int dataLen);

/**
* Upload configuration data to SPI. Low level API for cutils and clibtr.
*
* @param        target          - Target for upload:
*                                   RFPMG_TARGET           = 0x01
*                                   RFBAND_TARGET          = 0x02
*                                   ACCESS_PWD_TARGET      = 0x03
*                                   USER_KEY_TARGET        = 0x04
*                                   FLASH_TARGET           = 0x05
*                                   INTERNAL_EEPROM_TARGET = 0x06
*                                   EXTERNAL_EEPROM_TARGET = 0x07
*                                   SPECIAL_TARGET         = 0x08
* @param	dataToWrite	- data to be written to SPI
* @param	dataLen		- length (in bytes) of the data
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during read operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c SPI_IQRF_ERROR_CRCS = mismatched CRC
* @return	@c BASE_TYPES_OPER_OK = data were successfully written
*/
SPI_IQRF_DECLSPEC int spi_iqrf_upload(int target, const unsigned char *dataToWrite, unsigned int dataLen);

/**
* Download configuration data from SPI. Low level API for cutils and clibtr.
*
* @param        target          - Target for download:
*                                   RFPMG_TARGET           = 0x01
*                                   RFBAND_TARGET          = 0x02
*                                   ACCESS_PWD_TARGET      = 0x03
*                                   USER_KEY_TARGET        = 0x04
*                                   FLASH_TARGET           = 0x05
*                                   INTERNAL_EEPROM_TARGET = 0x06
*                                   EXTERNAL_EEPROM_TARGET = 0x07
*                                   SPECIAL_TARGET         = 0x08
* @param	dataToWrite	- data to be written to SPI
* @param	writeLen		- length (in bytes) of the data
* @param	readBuffer	- data are read to this buffer
* @param	readLen		- length (in bytes) of the data to read
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during read operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c SPI_IQRF_ERROR_CRCS = mismatched CRC
* @return	@c BASE_TYPES_OPER_OK = data were successfully read
*/
SPI_IQRF_DECLSPEC int spi_iqrf_download(int target, const unsigned char *dataToWrite, unsigned int writeLen, unsigned char *readBuffer, unsigned int readLen);

/**
* Enter programming mode
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during programming mode entry
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c BASE_TYPES_OPER_OK = Programming mode entry successfull
*/
SPI_IQRF_DECLSPEC int spi_iqrf_pe(void);

/**
* Terminate programming mode
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during programming mode termination
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c BASE_TYPES_OPER_OK = Programming mode termination successfull
*/

SPI_IQRF_DECLSPEC int spi_iqrf_pt(void);

/**
 * Terminates the library and frees up used resources.
 *
 * @return	@c BASE_TYPES_OPER_OK if operation has performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR if an error has occurred during operation.
 * @return	@c BASE_TYPES_LIB_NOT_INITIALIZED if the library has not been initialized.
 */
SPI_IQRF_DECLSPEC int spi_iqrf_destroy(void);

/**
* Returns information about last error or @c NULL, if no error has yet occurred.
* @return information about last error
* @return @c NULL if no error has yet occurred
*/
//extern errors_OperError* rpi_spi_iqrf_getLastError(void);

#ifdef __cplusplus
}
#endif

#endif // __SPI_IQRF_H
