/*
 * Copyright 2015 MICRORISC s.r.o.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

//TODO use std::chrono::high_resolution_clock for timing
#ifndef WIN32
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

#ifdef _SC_MONOTONIC_CLOCK
static uint64_t get_ms_ts()
{
    struct timespec ts;

    if (clock_gettime (CLOCK_MONOTONIC, &ts) == 0)
        return (uint64_t) (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    else
        return 0;
}
#else
static uint64_t get_ms_ts()
{
    struct timeval tv;

    if (gettimeofday (&tv, NULL) == 0)
        return (uint64_t) (tv.tv_sec * 1000 + tv.tv_usec / 1000);
    else
        return 0;
}
#endif

#else
//TODO this is really stupid solution - just to make it compilable on WIN
#include <windows.h>
#include <winioctl.h>
#include <errno.h>
#include <fcntl.h>
//#include <unistd.h>
#include <win/spidev.h>

#define _IOC_SIZEBITS   13
#define _IOC_DIRBITS    3

int ioctl(int fd, int rw, int* mode)
{
  return 0;
}

int nanosleep(int time)
{
  return 0;
}

//struct timespec {
//  unsigned tv_sec;
//  unsigned tv_nsec;
//};

static uint64_t get_ms_ts()
{
    LARGE_INTEGER counter, frequency;

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&counter);

	return counter.QuadPart / (frequency.QuadPart / 1000);
}

#endif

#define SPI_IQRF_EXPORTS

#include <spi_iqrf.h>
#include <sysfs_gpio.h>
#include <machines_def.h>
#include <sleepWrapper.h>

/************************************/
/* Private constants                */
/************************************/


/** Designates, that file descriptor is not used. */
static const int NO_FILE_DESCRIPTOR = -1;

/**
 * Device's mode (according to document: "SPI. Implementation in IQRF TR modules"): Idle clock
 * polarity: low Clock edge: output data on SCK rising edge.
 */
static const uint8_t SPI_MODE = SPI_MODE_0;

/** Device's wordsize. */
static const uint8_t BITS_PER_WORD = 8;


/** Device's max. bitrate [Hz]. */
static const uint32_t SPI_MAX_SPEED = 250000;

#define NANO_TO_MICRO_MULTIPLIER 1000
#define convetNanoToMicro(a)    (a * NANO_TO_MICRO_MULTIPLIER)

/** Time delay T1 between CS and SCLK falling edge for high speed communication. */
static const uint16_t DELAY_AFTER_CS_HS_US = 5;
/** Time delay T1 between CS and SCLK falling edge for low speed communication. */
static const uint16_t DELAY_AFTER_CS_LS_US = 10;

/** Time delay T2 in us after byte in high speed mode. */
static const uint16_t DELAY_AFTER_BYTE_HS_US = 150;
/** Time delay T2 in us after byte in low speed mode. */
//static const uint16_t DELAY_AFTER_BYTE_LS_US = 800000;

static const uint8_t CS_DESELECT_DEVICE = 0;
static const uint8_t CS_DEVICE_STAY_SELECTED = 1;

/** Delay between 2 consecutive bytes. T2 period */
static const struct timespec T2_DELAY = { .tv_sec = 0, .tv_nsec = 800000 };

/**
 * how long to wait [microseconds] after the last bit transfer and
 * before optionally deselecting the device before next transfer of
 * T1 period
 */
static const uint16_t DELAY_AFTER_TRANSFER = 10;

/** SPI checking packet indication. */
static const uint8_t SPI_IQRF_SPI_CHECK = 0x00;

/** SPI command packet indication. */
static const uint8_t SPI_IQRF_SPI_CMD = 0xF0;
static const uint8_t SPI_IQRF_SPI_CMD_TR_MODULE_INFO = 0xF5;
static const uint8_t SPI_IQRF_SPI_CMD_WRITE_TO_EEPROM = 0xF3;
static const uint8_t SPI_IQRF_SPI_CMD_READ_FROM_EEPROM = 0xF2;
static const uint8_t SPI_IQRF_SPI_CMD_WRITE_TO_EEEPROM = 0xF6;
static const uint8_t SPI_IQRF_SPI_CMD_WRITE_TO_FLASH = 0xF6;
static const uint8_t SPI_IQRF_SPI_CMD_READ_FROM_EEEPROM = 0xF6;
static const uint8_t SPI_IQRF_SPI_CMD_VERIFY_DATA_IN_FLASH = 0xFC;
static const uint8_t SPI_IQRF_SPI_CMD_UPLOAD_IQRF = 0xF9;

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

/** SPI device name size */
// Constant is used to specify array. Therefore, they can not be defined as static const uint8_t.
#define SPI_DEV_CAPACITY 4095

/** Defines how to work with IQRF communication buffer. */
typedef enum _spi_iqrf_SPICtype
{
  CTYPE_BUFFER_CHANGED,
  CTYPE_BUFFER_UNCHANGED
} spi_iqrf_SPICtype;


/************************************/
/* Private variables                */
/************************************/
/** Indicates, whether this library is initialized or not. 0 - not initialized 1 - initialized. */
static int libIsInitialized = 0;

/** Device file name*/
static char spiDev[SPI_DEV_CAPACITY+1];

/** File descriptor of the device special file. */
static int fd = -1;

/** Empty message used for delay T1 after the chip select signal. */
static struct spi_ioc_transfer nullTransfer;

/** Communication mode. Default value set to low speed communication mode. */
static spi_iqrf_CommunicationMode communicationMode = SPI_IQRF_LOW_SPEED_MODE;

/************************************/
/* Private functions predeclaration */
/************************************/
static int sendAndReceive(void *dataToSend, void *recvBuffer, unsigned int len);
static int sendAndReceiveLowSpeed(void *dataToSend, void *recvBuffer, unsigned int len);
static int sendAndReceiveHighSpeed(void *dataToSend, void *recvBuffer, unsigned int len);

/**
 * Initializes nullTransfer structure for low speed communication.
 * It is used for application of T1 delay - see document:
 * "SPI. Implementation if IQRF modules. User Guide", version: 130430.
 */
static void initNullTransferLowSpeed(void)
{
  nullTransfer.tx_buf = 0;
  nullTransfer.rx_buf = 0;
  nullTransfer.len = 0;
  nullTransfer.delay_usecs = DELAY_AFTER_CS_LS_US;
  nullTransfer.speed_hz = SPI_MAX_SPEED;
  nullTransfer.bits_per_word = BITS_PER_WORD;
  nullTransfer.cs_change = CS_DESELECT_DEVICE;
  nullTransfer.rx_nbits = 0;
  nullTransfer.tx_nbits = 0;
}

/**
* Initializes nullTransfer structure for high speed communication.
*/
static void initNullTransferHighSpeed(void)
{
  nullTransfer.tx_buf = (unsigned long)NULL;
  nullTransfer.rx_buf = (unsigned long)NULL;
  nullTransfer.len = 0;
  nullTransfer.delay_usecs = DELAY_AFTER_CS_HS_US;
  nullTransfer.speed_hz = SPI_MAX_SPEED;
  nullTransfer.bits_per_word = BITS_PER_WORD;
  nullTransfer.cs_change = CS_DESELECT_DEVICE;
  nullTransfer.rx_nbits = 0;
  nullTransfer.tx_nbits = 0;
}

/**
 * Sets SPI mode.
 *
 * @return	An nonnegative gpio_getValue = success
 * @return -1 = error.
 */
static int setMode()
{
  int setResult = 0;
  uint8_t rdMode = -1;

  setResult = ioctl(fd, SPI_IOC_WR_MODE, &SPI_MODE);
  if (setResult < 0)
  {
    return setResult;
  }

  setResult = ioctl(fd, SPI_IOC_RD_MODE, &rdMode);
  if (setResult < 0)
  {
    return setResult;
  }

  return 0;
}

/**
 * Sets bits per word.
 *
 * @return	An nonnegative gpio_getValue = success.
 * @return -1 = error.
 */
static int setBitsPerWord()
{
  int setResult = 0;
  uint8_t rdBits = -1;

  setResult = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &BITS_PER_WORD);
  if (setResult < 0)
  {
    return setResult;
  }

  setResult = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &rdBits);
  if (setResult < 0)
  {
    return setResult;
  }

  return 0;
}

/**
 * Sets maximum speed.
 *
 * @return	An nonnegative gpio_getValue = success
 * @return -1 = error.
 */
static int setMaxSpeed()
{
  int setResult = 0;
  uint8_t rdSpeed = -1;

  setResult = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_MAX_SPEED);
  if (setResult < 0)
  {
    return setResult;
  }

  setResult = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rdSpeed);
  if (setResult < 0)
  {
    return setResult;
  }

  return 0;
}

/**
 * Sends data stored in @c dataToSend buffer to SPI device.
 *
 * @param [in,out]	dataToSend	data to send.
 * @param [in]		len		  	length (in bytes) of data to send.
 *
 * @return	@c BASE_TYPES_OPER_OK, if operation is performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR, if some error occurred during operation.
 */
static int sendData(void *dataToSend, unsigned int len)
{
  uint8_t *dummyData;
  int32_t operResult;

  dummyData = malloc(len * sizeof(uint8_t));

  operResult = sendAndReceive(dataToSend, dummyData, len);

  free(dummyData);

  return operResult;
}

/**
 * Sends data stored in @c dataToSend buffer to SPI device and receives data from SPI device
 * to @c recvBuffer buffer. Maximal length of both buffers must not exceed @c len.
 *
 * @param [in,out]	dataToSend	data to send.
 * @param [in,out]	recvBuffer	buffer to store received data.
 * @param [in]		len		  	length (in bytes) of buffers.
 *
 * @return	@c BASE_TYPES_OPER_OK = operation is performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR = some error occurred during operation.
 */
static int sendAndReceive(void *dataToSend, void *recvBuffer, unsigned int len)
{
  if (communicationMode == SPI_IQRF_LOW_SPEED_MODE)
    return sendAndReceiveLowSpeed(dataToSend, recvBuffer, len);
  return sendAndReceiveHighSpeed(dataToSend, recvBuffer, len);
}

/**
* Sends data with low speed setting stored in @c dataToSend buffer to SPI device and receives data from SPI device
* to @c recvBuffer buffer. Maximal length of both buffers must not exceed @c len.
*
* @param [in,out]	dataToSend	data to send.
* @param [in,out]	recvBuffer	buffer to store received data.
* @param [in]		len		  	length (in bytes) of buffers.
*
* @return	@c BASE_TYPES_OPER_OK = operation performs successfully.
* @return	@c BASE_TYPES_OPER_ERROR = some error occurred during operation.
*/
static int sendAndReceiveLowSpeed(void *dataToSend, void *recvBuffer, unsigned int len)
{
  int result = 0;
  int trans_id = 0;
  uint8_t* tx = NULL;
  uint8_t* rx = NULL;
  struct spi_ioc_transfer completeTransfer[2];

  tx = malloc(len * sizeof(uint8_t));
  memcpy(tx, dataToSend, len * sizeof(uint8_t));

  rx = malloc(len * sizeof(uint8_t));
  memset(rx, 0, len * sizeof(uint8_t));

  struct spi_ioc_transfer dataTransfer = {
    .tx_buf = (unsigned long)tx,
    .rx_buf = (unsigned long)rx,
    .len = 1,
    .delay_usecs = DELAY_AFTER_CS_LS_US,
    .speed_hz = SPI_MAX_SPEED,
    .bits_per_word = BITS_PER_WORD,
    .cs_change = CS_DESELECT_DEVICE,
    .tx_nbits = 0,
    .rx_nbits = 0
  };

  completeTransfer[0] = nullTransfer;
  completeTransfer[1] = dataTransfer;

  for (trans_id = 0; trans_id < len; trans_id++) {
    completeTransfer[1].tx_buf = (unsigned long)&(tx[trans_id]);
    completeTransfer[1].rx_buf = (unsigned long)&(rx[trans_id]);

    result = ioctl(fd, SPI_IOC_MESSAGE(2), &completeTransfer);
    if (result == -1) {
      goto end;
    }

    // delay T3
    int sleepResult = nanosleep(&T2_DELAY, NULL);
    if (sleepResult == -1) {
      goto end;
    }
  }

  memcpy(recvBuffer, rx, len * sizeof(uint8_t));

end:
  free(tx);
  free(rx);

  return result;
}

/**
* Sends data with high speed setting stored in @c dataToSend buffer to SPI device and receives data from SPI device
* to @c recvBuffer buffer. Maximal length of both buffers must not exceed @c len.
*
* @param [in,out]	dataToSend	data to send.
* @param [in,out]	recvBuffer	buffer to store received data.
* @param [in]		len		  	length (in bytes) of buffers.
*
* @return	@c BASE_TYPES_OPER_OK = operation performs successfully.
* @return	@c BASE_TYPES_OPER_ERROR = some error occurred during operation.
*/
static int sendAndReceiveHighSpeed(void *dataToSend, void *recvBuffer, unsigned int len)
{
  int result = 0;
  int trans_id = 0;
  uint8_t *tx = NULL;
  uint8_t *rx = NULL;
  struct spi_ioc_transfer completeTransfer[2];

  tx = malloc(len * sizeof(uint8_t));
  memcpy(tx, dataToSend, len * sizeof(uint8_t));

  rx = malloc(len * sizeof(uint8_t));
  memset(rx, 0, len * sizeof(uint8_t));

  struct spi_ioc_transfer dataTransfer = {
    .tx_buf = (unsigned long)tx,
    .rx_buf = (unsigned long)rx,
    .len = 1,
    .delay_usecs = DELAY_AFTER_BYTE_HS_US,
    .speed_hz = SPI_MAX_SPEED,
    .bits_per_word = BITS_PER_WORD,
    .cs_change = CS_DESELECT_DEVICE,
    .tx_nbits = 0,
    .rx_nbits = 0
  };

  completeTransfer[0] = nullTransfer;
  completeTransfer[1] = dataTransfer;

  if (len > 1)
    completeTransfer[1].cs_change = CS_DEVICE_STAY_SELECTED;

  // first byte
  result = ioctl(fd, SPI_IOC_MESSAGE(2), &completeTransfer);
  if (result < 0)
  {
    goto end;
  }

  ++trans_id;

  for (; trans_id < len; trans_id++)
  {
    completeTransfer[1].tx_buf = (unsigned long)&(tx[trans_id]);
    completeTransfer[1].rx_buf = (unsigned long)&(rx[trans_id]);

    if (trans_id == (len - 1))
    {
      completeTransfer[1].delay_usecs = DELAY_AFTER_CS_HS_US;
      completeTransfer[1].cs_change = CS_DESELECT_DEVICE;
    }

    result = ioctl(fd, SPI_IOC_MESSAGE(1), &completeTransfer[1]);
    if (result < 0)
    {
      goto end;
    }
  }

  memcpy(recvBuffer, rx, len * sizeof(uint8_t));

end:
  free(tx);
  free(rx);

  return result;
}

/*
 * Checks SPI Data Ready flag
 *
 * @param	spiStatus	SPI status
 *
 * @return 1 = SPI Data Ready
 * @return 0 = No SPI Data Ready signal
 */
static int isSPIDataReady(uint8_t spiStatus)
{
  if ((spiStatus >= 0x40) && (spiStatus < SPI_IQRF_SPI_READY_COMM))
  {
    return 1;
  }
  return 0;
}

/*
 * Sets PTYPE of message packet.
 *
 * @param	pType		PTYPE of message
 * @param	ctype		CTYPE of message
 * @param	dataLen		length (in bytes) of the data
 *
 */
static void setPTYPE(uint8_t *pType, spi_iqrf_SPICtype ctype, unsigned int dataLen)
{
  *pType = dataLen;
  if (ctype == CTYPE_BUFFER_CHANGED)
  {
    *pType |= 128;
  }
}

/*
* Exclusive OR of all data stored in buffer.
*
* @param	crcm		calculated CRCM
* @param	buffer		buffer with input data for XOR calculation
* @param	buffSize	size of the buffer with data
*
*/
static void bufferXor(uint8_t *crcm, uint8_t *buffer, unsigned int buffSize)
{
  uint8_t dataId = 0;

  for (dataId = 0; dataId < buffSize; dataId++)
  {
    *crcm ^= buffer[dataId];
  }
}

/*
 * Calculates and returns CRCM.
 *
 * @param	ptype		PTYPE of message
 * @param	data		buffer with input data for CRC calculation
 * @param	dataLen		length (in bytes) of the data
 *
 * @return CRCM
 */
static uint8_t getCRCM(uint8_t ptype, uint8_t *data, unsigned int dataLen)
{
  uint8_t crcm = 0;

  crcm ^= SPI_IQRF_SPI_CMD;
  crcm ^= ptype;
  bufferXor(&crcm, data, dataLen);
  crcm ^= 0x5F;

  return crcm;
}

/*
 * Calculates and returns CRCS.
 *
 * @param	ptype		PTYPE of message
 * @param	data		buffer with input data for CRC calculation
 * @param	dataLen		length (in bytes) of the data
 *
 * @return CRCS
 */
static uint8_t getCRCS(uint8_t ptype, uint8_t *data, unsigned int dataLen)
{
  uint8_t crcs = 0;

  crcs ^= ptype;
  bufferXor(&crcs, data, dataLen);
  crcs ^= 0x5F;

  return crcs;
}

/*
 * Verify CRCS.
 *
 * @param	ptype		PTYPE of message
 * @param	recvData	input data for CRC verification
 * @param	dataLen		length (in bytes) of the data
 *
 * @return 1 = CRCS OK
 * @return 0 = wrong CRCS
 */
static int verifyCRCS(uint8_t ptype, uint8_t *recvData, unsigned int dataLen,
  uint8_t crcsToVerify
  )
{
  uint8_t countedCrcs = 0;

  countedCrcs = getCRCS(ptype, recvData, dataLen);
  if (crcsToVerify == countedCrcs)
  {
    return 1;
  }
  return 0;
}

/**
 * Checks SPI status gpio_getValue.
 *
 * @param	spiStatus	The SPI status obtained from module.
 *
 * @return	Returns 1 if the specified SPI status indicates valid gpio_getValue of IQRF_SPIStatus_DataNotReady enum.
 */
static int isSPINoDataReady(uint8_t spiStatus)
{
  switch (spiStatus)
  {
  case SPI_IQRF_SPI_DISABLED:
  case SPI_IQRF_SPI_SUSPENDED:
  case SPI_IQRF_SPI_BUFF_PROTECT:
  case SPI_IQRF_SPI_CRCM_ERR:
  case SPI_IQRF_SPI_READY_COMM:
  case SPI_IQRF_SPI_READY_PROG:
  case SPI_IQRF_SPI_READY_DEBUG:
  case SPI_IQRF_SPI_SLOW_MODE:
  case SPI_IQRF_SPI_HW_ERROR:
    return 1;
  default:
    return 0;
  }
}

/**
 * Checks length of the data to be written or read.
 *
 * @param	dataLen - Length (in bytes) of the data.
 *
 * @return	@c BASE_TYPES_OPER_ERROR if @dataLen is bellow or equal zero or is higher than @c SPI_IQRF_MAX_DATA_LENGTH.
 */
static int checkDataLen(unsigned int dataLen)
{
  if (dataLen <= 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  if (dataLen > SPI_IQRF_MAX_DATA_LENGTH)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  return BASE_TYPES_OPER_OK;
}

/**
* Initialization of the SPI interface for communication with IQRF module.
*
* @param	dev - Device with SPI interface
*
* @return	@c BASE_TYPES_OPER_ERROR = initialization failed
* @return	@c BASE_TYPES_OPER_OK = initialization was correct
*/
int spi_iqrf_init(const char *dev)
{
  int32_t ioResult = 0;
  int32_t initResult = 0;

  if (libIsInitialized == 1)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  while (1) {
  // Bind all SPI pins to SPI controler
  if (iqrf_switch_pin_driver(RPIIO_PIN_CE0 , PIN_SPI) != 0) {
      break;
  }
  if (iqrf_switch_pin_driver(MISO_GPIO , PIN_SPI) != 0) {
      break;
  }
  if (iqrf_switch_pin_driver(MOSI_GPIO , PIN_SPI) != 0) {
      break;
  }
  if (iqrf_switch_pin_driver(SCLK_GPIO , PIN_SPI) != 0) {
      break;
  }

  // Copy SPI device name
  if (strlen(dev) > SPI_DEV_CAPACITY) {
      break;
  } else {
      strcpy(spiDev, dev);
  }

  // enable PWR for TR communication
  initResult = gpio_setup(RESET_GPIO, GPIO_DIRECTION_OUT, 1);
  if (initResult < 0)
  {
	break;
	//return BASE_TYPES_OPER_ERROR;
  }

  spi_iqrf_setCommunicationMode(SPI_IQRF_LOW_SPEED_MODE);

  if (fd != NO_FILE_DESCRIPTOR)
  {
	break;
	//return BASE_TYPES_OPER_ERROR;
  }

  fd = open(dev, O_RDWR);
  if (fd < 0)
  {
	break;
	//return BASE_TYPES_OPER_ERROR;
  }

  // set SPI mode
  initResult = setMode();
  if (initResult < 0)
  {
	break;
	//return BASE_TYPES_OPER_ERROR;
  }

  // set bits per word
  initResult = setBitsPerWord();
  if (initResult < 0)
  {
	break;
	//return BASE_TYPES_OPER_ERROR;
  }

  // set max. speed
  initResult = setMaxSpeed();
  if (initResult < 0)
  {
	break;
	//return BASE_TYPES_OPER_ERROR;
  }

    libIsInitialized = 1;
    break;
  }

  if (libIsInitialized) {
	return BASE_TYPES_OPER_OK;
  }
  else {
	gpio_cleanup(RESET_GPIO);
    return BASE_TYPES_OPER_ERROR;
  }

}

/**
* Initialization of the SPI for IQRF with default settings.
*
* @return	@c BASE_TYPES_OPER_ERROR = initialization failed
* @return	@c BASE_TYPES_OPER_OK = initialization was correct
*/
int spi_iqrf_initDefault()
{
  return spi_iqrf_init(SPI_IQRF_DEFAULT_SPI_DEVICE);
}

/**
* Gets current communication mode
*
* @return	@c BASE_TYPES_OPER_ERROR = unknown communication mode
* @return	@c SPI_IQRF_LOW_SPEED_MODE = low speed communication mode
* @return	@c SPI_IQRF_HIGH_SPEED_MODE = high speed communication mode
*/
spi_iqrf_CommunicationMode spi_iqrf_getCommunicationMode()
{
  return communicationMode;
}

/**
* Sets communication mode
*
* @param	communication_mode	required communication mode.
*
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED =  SPI library is not initialized
* @return	@c BASE_TYPES_OPER_ERROR = error during mode setting operation
* @return	@c BASE_TYPES_OPER_OK = mode successfully set
*/
int spi_iqrf_setCommunicationMode(spi_iqrf_CommunicationMode communication_mode)
{
  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  switch (communicationMode)
  {
  case SPI_IQRF_LOW_SPEED_MODE:
    initNullTransferLowSpeed();
    break;
  case SPI_IQRF_HIGH_SPEED_MODE:
    initNullTransferHighSpeed();
    break;
  default:
    return BASE_TYPES_OPER_ERROR;
  }

  communicationMode = communication_mode;

  return BASE_TYPES_OPER_OK;
}

/**
* Gets SPI status
*
* @param	spiStatus	current SPI status
*
* @return	@c BASE_TYPES_OPER_ERROR = error during SPI status reading
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c SPI_IQRF_ERROR_BAD_STATUS = wrong SPI status response
* @return	@c BASE_TYPES_OPER_OK = SPI status reading was correct
*/
int spi_iqrf_getSPIStatus(spi_iqrf_SPIStatus *spiStatus)
{
  uint8_t spiCheck = SPI_IQRF_SPI_CHECK;
  uint8_t spiResultStat = 0;
  int checkResult;

  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (spiStatus == NULL)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  if (fd < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  checkResult = sendAndReceive((void *)&spiCheck, (void *)&spiResultStat, 1);
  if (checkResult < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  // if checking is OK
  if (isSPIDataReady(spiResultStat))
  {
    spiStatus->isDataReady = 1;
    if (spiResultStat == 0x40)
    {
      spiStatus->dataReady = 64;
    }
    else
    {
      spiStatus->dataReady = spiResultStat - 0x40;
    }
    return BASE_TYPES_OPER_OK;
  }

  // check, if the return gpio_getValue of SPI status is correct
  if (isSPINoDataReady(spiResultStat))
  {
    spiStatus->isDataReady = 0;
    spiStatus->dataNotReadyStatus = spiResultStat;
    return BASE_TYPES_OPER_OK;
  }
  return SPI_IQRF_ERROR_BAD_STATUS;
}

/**
* Writes data to SPI
*
* @param	dataToWrite	- data to be written to SPI
* @param	dataLen		- length (in bytes) of the data
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during write operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c SPI_IQRF_ERROR_BAD_STATUS = wrong SPI status response
* @return	@c BASE_TYPES_OPER_OK = data was successfully written
*/
int spi_iqrf_write(void *dataToWrite, unsigned int dataLen)
{
  uint8_t *dataToSend = NULL;
  uint8_t ptype = 0;
  uint8_t crcm = 0;
  uint8_t sendResult = 0;
  int dataLenCheckRes = BASE_TYPES_OPER_ERROR;

  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  // checking input parameters
  if (dataToWrite == NULL)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dataLenCheckRes = checkDataLen(dataLen);
  if (dataLenCheckRes)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dataToSend = malloc((dataLen + 3) * sizeof(uint8_t));

  // set command indication
  dataToSend[0] = SPI_IQRF_SPI_CMD;

  // set PTYPE
  setPTYPE(&ptype, CTYPE_BUFFER_CHANGED, dataLen);
  dataToSend[1] = ptype;

  // copy data
  memcpy(dataToSend + 2, dataToWrite, dataLen);

  // set crcm
  crcm = getCRCM(ptype, dataToWrite, dataLen);
  dataToSend[dataLen + 2] = crcm;

  // send data to module
  sendResult = sendData(dataToSend, dataLen + 3);
  free(dataToSend);
  if (sendResult < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  return 0;
}

/**
* Reads data from SPI
*
* @param	readBuffer	- data are read to this buffer
* @param	dataLen		- length (in bytes) of the data
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during read operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c SPI_IQRF_ERROR_CRCS = mismatched CRC
* @return	@c BASE_TYPES_OPER_OK = data were successfully read
*/
int spi_iqrf_read(void *readBuffer, unsigned int dataLen)
{
  uint8_t *dummyData = NULL;
  uint8_t *receiveBuffer = NULL;
  uint8_t ptype = 0;
  uint8_t crcm = 0;
  uint8_t sendResult = 0;
  int dataLenCheckRes = -1;

  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  // checking input parameters
  if (readBuffer == NULL)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dataLenCheckRes = checkDataLen(dataLen);
  if (dataLenCheckRes)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dummyData = malloc((dataLen + 3) * sizeof(uint8_t));
  receiveBuffer = malloc((dataLen + 3) * sizeof(uint8_t));

  // set command indication
  dummyData[0] = SPI_IQRF_SPI_CMD;

  // set PTYPE
  setPTYPE(&ptype, CTYPE_BUFFER_UNCHANGED, dataLen);
  dummyData[1] = ptype;

  // dummy values
  memset(dummyData + 2, 0, dataLen);

  // set crcm
  crcm = getCRCM(ptype, dummyData + 2, dataLen);
  dummyData[dataLen + 2] = crcm;

  // send data to module
  sendResult = sendAndReceive(dummyData, receiveBuffer, dataLen + 3);
  free(dummyData);
  if (sendResult < 0)
  {
    free(receiveBuffer);
    return BASE_TYPES_OPER_ERROR;
  }

  // verify CRCS
  if (!verifyCRCS(ptype, receiveBuffer + 2, dataLen, receiveBuffer[dataLen + 2]))
  {
    free(receiveBuffer);
    return SPI_IQRF_ERROR_CRCS;
  }

  // copy received data into user buffer
  memcpy(readBuffer, receiveBuffer + 2, dataLen);
  free(receiveBuffer);

  return BASE_TYPES_OPER_OK;
}

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
int spi_iqrf_get_tr_module_info(void *readBuffer, unsigned int dataLen)
{
  uint8_t *dummyData = NULL;
  uint8_t *receiveBuffer = NULL;
  uint8_t ptype = 0;
  uint8_t crcm = 0;
  uint8_t sendResult = 0;
  int dataLenCheckRes = -1;

  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  // checking input parameters
  if (readBuffer == NULL)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dataLenCheckRes = checkDataLen(dataLen);
  if (dataLenCheckRes || dataLen < 16)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dummyData = malloc((dataLen + 3) * sizeof(uint8_t));
  if (dummyData == NULL) {
       return BASE_TYPES_OPER_ERROR;
  }
  
  receiveBuffer = malloc((dataLen + 3) * sizeof(uint8_t));
  if (receiveBuffer == NULL) {
       return BASE_TYPES_OPER_ERROR;
  }

  // set command indication
  dummyData[0] = SPI_IQRF_SPI_CMD_TR_MODULE_INFO;

  // set PTYPE
  setPTYPE(&ptype, CTYPE_BUFFER_UNCHANGED, dataLen);
  dummyData[1] = ptype;

  // dummy values
  memset(dummyData + 2, 0, dataLen);

  // set crcm
  crcm = getCRCM(ptype, dummyData + 2, dataLen);
  dummyData[dataLen + 2] = crcm;

  // send data to module
  sendResult = sendAndReceive(dummyData, receiveBuffer, dataLen + 3);
  free(dummyData);
  if (sendResult < 0)
  {
    free(receiveBuffer);
    return BASE_TYPES_OPER_ERROR;
  }

  // verify CRCS
  if (!verifyCRCS(ptype, receiveBuffer + 2, dataLen, receiveBuffer[dataLen + 2]))
  {
    free(receiveBuffer);
    return SPI_IQRF_ERROR_CRCS;
  }

  // copy received data into user buffer
  memcpy(readBuffer, receiveBuffer + 2, dataLen);
  free(receiveBuffer);

  return BASE_TYPES_OPER_OK;
}

static void get_eeeprom_blk_wr_addr(uint8_t *dst, const uint8_t *src)
{
    uint16_t val = 0;
    
    val = src[1];
    val << 8;
    val += src[0];
    
    val = (val - 0x0200) / 0x20;
    
    dst[0] = val & 0x00ff;
    dst[1] = val >> 8;
}


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
* @return	@c BASE_TYPES_OPER_OK = data were successfully read
*/
int spi_iqrf_upload(int target, const unsigned char *dataToWrite, unsigned int dataLen)
{
  uint8_t *dataToSend = NULL;
  uint8_t ptype = 0;
  uint8_t crcm = 0;
  uint8_t sendResult = 0;
  int dataLenCheckRes = BASE_TYPES_OPER_ERROR;
  int offset;

  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  // checking input parameters
  if (dataToWrite == NULL)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dataLenCheckRes = checkDataLen(dataLen);
  if (dataLenCheckRes)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  // Set offset for aditional data for buffer allocation
  switch (target) {
      case RFPMG_TARGET:
      case RFBAND_TARGET:
      case ACCESS_PWD_TARGET:
      case USER_KEY_TARGET:
          offset = 2;
          break;
      default:
          offset = 0;
          break;
  }
  
  dataToSend = malloc((dataLen + offset + 3) * sizeof(uint8_t));
  if (dataToSend == NULL) {
       return BASE_TYPES_OPER_ERROR;
  }

  // set command indication
  switch (target) {
      case CFG_TARGET:
          // CFG_TARGET: is unsupported, configuration upload must be split into separate memory uploads.
          free(dataToSend);
          return BASE_TYPES_OPER_ERROR;
          break;
      case RFPMG_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_WRITE_TO_EEPROM;
          dataToSend[2] = 0xC1;
          dataToSend[3] = 0x01;
          dataToSend[4] = dataToWrite[0];
          break;
      case RFBAND_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_WRITE_TO_EEPROM;
          dataToSend[2] = 0xC0;
          dataToSend[3] = 0x01;
          dataToSend[4] = dataToWrite[0];
          break;
      case ACCESS_PWD_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_WRITE_TO_EEPROM;
          dataToSend[2] = 0xD0;
          dataToSend[3] = 0x10;
          // copy data
          memcpy(dataToSend + 4, dataToWrite, dataLen);
          break;
      case USER_KEY_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_WRITE_TO_EEPROM;
          dataToSend[2] = 0xD1;
          dataToSend[3] = 0x10;
          // copy data
          memcpy(dataToSend + 4, dataToWrite, dataLen);
          break;
      case FLASH_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_WRITE_TO_FLASH;
          // copy data
          memcpy(dataToSend + 2, dataToWrite, dataLen);
          break;
      case INTERNAL_EEPROM_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_WRITE_TO_EEPROM;
          // Only low 8b of address are used for addressing internal eeprom 
          dataToSend[2] = dataToWrite[0];
          dataToSend[3] = dataLen - 2;
          // copy data
          memcpy(dataToSend + 4, dataToWrite + 2 , dataLen - 2);
          break;
      case EXTERNAL_EEPROM_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_WRITE_TO_EEEPROM;
          // Get external eeprom block address from byte address
          get_eeeprom_blk_wr_addr(&dataToSend[2],  dataToWrite);
          // copy data
          memcpy(dataToSend + 4, dataToWrite + 2, dataLen - 2);
          break;
      case SPECIAL_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_UPLOAD_IQRF;
          // copy data
          memcpy(dataToSend + 2, dataToWrite, dataLen);
          break;
      default:
          free(dataToSend);
          return BASE_TYPES_OPER_ERROR;
          break;
  }
  
  // set PTYPE
  setPTYPE(&ptype, CTYPE_BUFFER_CHANGED, dataLen);
  dataToSend[1] = ptype;

  // set crcm
  crcm = getCRCM(ptype, dataToWrite, dataLen);
  dataToSend[dataLen + 2] = crcm;

  // send data to module
  sendResult = sendData(dataToSend, dataLen + 3);
  free(dataToSend);
  if (sendResult < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  return 0;
  

}

static void get_eeeprom_blk_rd_addr(uint8_t *dst, const uint8_t *src)
{
    uint16_t val = 0;
    
    val = src[1];
    val << 8;
    val += src[0];
    
    val = (val / 0x20) + 0x0400;
    
    dst[0] = val & 0x00ff;
    dst[1] = val >> 8;
}

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
int spi_iqrf_download(int target, const unsigned char *dataToWrite, unsigned int writeLen, unsigned char *readBuffer, unsigned int readLen)
{
  uint8_t *dataToSend = NULL;
  uint8_t tmpBuffer[32];
  uint8_t ptype = 0;
  uint8_t crcm = 0;
  uint8_t sendResult = 0;
  int dataLenCheckRes = BASE_TYPES_OPER_ERROR;
  spi_iqrf_SPIStatus status;
  
  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  // checking input parameters
  if (dataToWrite == NULL)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  if (target == CFG_TARGET || target == RFPMG_TARGET || target == RFPMG_TARGET) {
    writeLen = 2;
  }
  
  dataLenCheckRes = checkDataLen(writeLen);
  if (dataLenCheckRes || readLen > 32)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  dataToSend = malloc((writeLen + 3) * sizeof(uint8_t));
  
  if (dataToSend == NULL) {
      return BASE_TYPES_OPER_ERROR;
  }

  // set command indication
  switch (target) {
      case CFG_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_VERIFY_DATA_IN_FLASH;
          dataToSend[2] = 0xC0;
          dataToSend[3] = 0x37;
          return BASE_TYPES_OPER_ERROR;
          break;
      case RFPMG_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_READ_FROM_EEPROM;
          dataToSend[2] = 0xC0;
          dataToSend[3] = 0x0;
          break;
      case RFBAND_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_READ_FROM_EEPROM;
          dataToSend[2] = 0xC0;
          dataToSend[3] = 0x0;
          break;
      case ACCESS_PWD_TARGET:
          // ACCESS_PWD_TARGET: is unsupported, access password can not be downloaded.
          free(dataToSend);
          return BASE_TYPES_OPER_ERROR;
          break;
      case USER_KEY_TARGET:
          // USER_KEY_TARGET: is unsupported, user key can not be downloaded.
          free(dataToSend);
          return BASE_TYPES_OPER_ERROR;
          break;
      case FLASH_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_VERIFY_DATA_IN_FLASH;
          // copy data
          memcpy(dataToSend + 2, dataToWrite, writeLen);
          break;
      case INTERNAL_EEPROM_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_READ_FROM_EEPROM;
          // Only low 8b of address are used for addressing internal eeprom 
          dataToSend[2] = dataToWrite[0];
          dataToSend[3] = 0;
          break;
      case EXTERNAL_EEPROM_TARGET:
          dataToSend[0] = SPI_IQRF_SPI_CMD_READ_FROM_EEEPROM;
          // copy data
          memcpy(dataToSend + 2, dataToWrite, writeLen);
          break;
      case SPECIAL_TARGET:
          // SPECIAL_TARGET: is unsupported, user key can not be downloaded.
          free(dataToSend);
          return BASE_TYPES_OPER_ERROR;
      default:
          free(dataToSend);
          return BASE_TYPES_OPER_ERROR;
          break;
  }
  
  // set PTYPE
  setPTYPE(&ptype, CTYPE_BUFFER_CHANGED, writeLen);
  dataToSend[1] = ptype;

  // set crcm
  crcm = getCRCM(ptype, dataToWrite, writeLen);
  dataToSend[writeLen + 2] = crcm;

  // send data to module
  sendResult = sendData(dataToSend, writeLen + 3);
  free(dataToSend);
  if (sendResult < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  status.dataReady = 0;
  status.isDataReady = 0;
  while (!(status.isDataReady == 1) && (status.dataReady == 32)) {
    int retval = spi_iqrf_getSPIStatus(&status);
    if (BASE_TYPES_OPER_OK != retval) {
      return BASE_TYPES_OPER_ERROR;
    }
  }
  
  if (spi_iqrf_read(tmpBuffer, 32) != BASE_TYPES_OPER_OK) {
      return BASE_TYPES_OPER_ERROR;
  }
  
  switch (target) {
      case RFPMG_TARGET:
          readBuffer[0] = tmpBuffer[1];
          break;
      case RFBAND_TARGET:
          readBuffer[0] = tmpBuffer[0];
          break;
      default:
           memcpy(readBuffer, tmpBuffer, readLen);
  }
  return 0;
}

/**
* Reset TR. This internal function has many side effects. Ports are controled by GPIO, all are output ports, ...
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during TR reset
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c BASE_TYPES_OPER_OK = TR reset successfull
*/
static int spi_reset_tr()
{
  if (spi_iqrf_destroy() != BASE_TYPES_OPER_OK) 
  {
      return BASE_TYPES_OPER_ERROR;
  }
  
  if (iqrf_switch_pin_driver(RPIIO_PIN_CE0 , PIN_GPIO) != 0) {
        return BASE_TYPES_OPER_ERROR;
  }
  if (iqrf_switch_pin_driver(MISO_GPIO , PIN_GPIO) != 0) {
      return BASE_TYPES_OPER_ERROR;
  }
  if (iqrf_switch_pin_driver(MOSI_GPIO , PIN_GPIO) != 0) {
      return BASE_TYPES_OPER_ERROR;
  }
  if (iqrf_switch_pin_driver(SCLK_GPIO , PIN_GPIO) != 0) {
      return BASE_TYPES_OPER_ERROR;
  }
  
  // Set all SPI pins to low
  if (gpio_setup(RPIIO_PIN_CE0, GPIO_DIRECTION_OUT, 0) < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }
  if (gpio_setup(MISO_GPIO, GPIO_DIRECTION_OUT, 0) < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }
  if (gpio_setup(MOSI_GPIO, GPIO_DIRECTION_OUT, 0) < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }
  if (gpio_setup(SCLK_GPIO, GPIO_DIRECTION_OUT, 0) < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }
  
  // Disable PWR for TR
  if (gpio_setup(RESET_GPIO, GPIO_DIRECTION_OUT, 0) < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }
  
  // Sleep for 300ms
  SLEEP(300);
  
  // Enable PWR for TR
  if (gpio_setup(RESET_GPIO, GPIO_DIRECTION_OUT, 1) < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }
  
  // Set SPI pins to idle
  if (gpio_setValue(RPIIO_PIN_CE0, 1) < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }
  
  return BASE_TYPES_OPER_OK;
}


/**
* Enter programming mode
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during programming mode entry
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c BASE_TYPES_OPER_OK = Programming mode entry successfull
*/
int spi_iqrf_pe(void)
{
    uint64_t start;
    spi_iqrf_SPIStatus status;
    
    if (spi_iqrf_getSPIStatus(&status) != BASE_TYPES_OPER_OK)
    {
        return BASE_TYPES_OPER_ERROR;
    }
    if (status.dataNotReadyStatus == SPI_IQRF_SPI_READY_PROG) {
        return BASE_TYPES_OPER_OK;
    }
    
    if (spi_reset_tr() != BASE_TYPES_OPER_OK)
    {
        goto pe_error;
    }
    
    if (gpio_setDirection(MISO_GPIO, GPIO_DIRECTION_OUT) < 0)
    {
        goto pe_error;
    }
    
    if (gpio_setDirection(MOSI_GPIO, GPIO_DIRECTION_IN) < 0)
    {
        goto pe_error;
    }
    
    start = get_ms_ts();
    while (start - get_ms_ts() < 410) {
        if (gpio_setValue(MISO_GPIO, gpio_getValue(MOSI_GPIO)) < 0)
        {
            goto pe_error;
        }
    }
    
    if (iqrf_switch_pin_driver(RPIIO_PIN_CE0 , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    if (iqrf_switch_pin_driver(MISO_GPIO , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    if (iqrf_switch_pin_driver(MOSI_GPIO , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    if (iqrf_switch_pin_driver(SCLK_GPIO , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    
    // Init SPI
    if (spi_iqrf_init(spiDev) != BASE_TYPES_OPER_OK)
    {
        return BASE_TYPES_OPER_ERROR;
    }
    
    status.dataNotReadyStatus = SPI_IQRF_SPI_DISABLED;
    status.isDataReady = 0;
    start = get_ms_ts();
    while (start - get_ms_ts() < 1000) {
        if (spi_iqrf_getSPIStatus(&status) != BASE_TYPES_OPER_OK)
        {
            return BASE_TYPES_OPER_ERROR;
        }
        
        if (status.dataNotReadyStatus == SPI_IQRF_SPI_READY_PROG) {
            break;
        }
    }
    if (status.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG) {
        return BASE_TYPES_OPER_ERROR;
    }
    return BASE_TYPES_OPER_OK;
    
pe_error:
    iqrf_switch_pin_driver(RPIIO_PIN_CE0 , PIN_SPI);
    iqrf_switch_pin_driver(MISO_GPIO , PIN_SPI);
    iqrf_switch_pin_driver(MOSI_GPIO , PIN_SPI);
    iqrf_switch_pin_driver(SCLK_GPIO , PIN_SPI);
    return BASE_TYPES_OPER_ERROR;
}

/**
* Terminate programming mode
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during programming mode termination
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c BASE_TYPES_OPER_OK = Programming mode termination successfull
*/

int spi_iqrf_pt(void)
{
    uint64_t start;
    spi_iqrf_SPIStatus status;
    
    if (spi_iqrf_getSPIStatus(&status) != BASE_TYPES_OPER_OK)
    {
        return BASE_TYPES_OPER_ERROR;
    }
    if (status.dataNotReadyStatus == SPI_IQRF_SPI_READY_COMM) {
        return BASE_TYPES_OPER_OK;
    }
    
    status.dataNotReadyStatus = SPI_IQRF_SPI_DISABLED;
    status.isDataReady = 0;
    start = get_ms_ts();
    while (start - get_ms_ts() < 1000) {
        if (spi_iqrf_getSPIStatus(&status) != BASE_TYPES_OPER_OK)
        {
            return BASE_TYPES_OPER_ERROR;
        }
        
        if (status.dataNotReadyStatus == SPI_IQRF_SPI_READY_PROG) {
            break;
        }
    }
    if (status.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG) {
        return BASE_TYPES_OPER_ERROR;
    }
    
    if (spi_reset_tr() != BASE_TYPES_OPER_OK)
    {
        iqrf_switch_pin_driver(RPIIO_PIN_CE0 , PIN_SPI);
        iqrf_switch_pin_driver(MISO_GPIO , PIN_SPI);
        iqrf_switch_pin_driver(MOSI_GPIO , PIN_SPI);
        iqrf_switch_pin_driver(SCLK_GPIO , PIN_SPI);
        return BASE_TYPES_OPER_ERROR;
    }
    
    if (iqrf_switch_pin_driver(RPIIO_PIN_CE0 , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    if (iqrf_switch_pin_driver(MISO_GPIO , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    if (iqrf_switch_pin_driver(MOSI_GPIO , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    if (iqrf_switch_pin_driver(SCLK_GPIO , PIN_SPI) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
    
    // Init SPI
    if (spi_iqrf_init(spiDev) != BASE_TYPES_OPER_OK)
    {
        return BASE_TYPES_OPER_ERROR;
    }
}


/**
* Destroys IQRF SPI library object and releases SPI port
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during SPI destroy operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c BASE_TYPES_OPER_OK = SPI was successfully destroyed
*/
int spi_iqrf_destroy(void)
{
  int ioDestroyRes = -1;
  int closeRes = -1;

  if (libIsInitialized == 0)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  // after calling this method, the behavior of the library will be
  // like if the library was not initialized
  libIsInitialized = 0;

  // destroy used rpi_io library
  gpio_cleanup(RESET_GPIO);

  if (fd == NO_FILE_DESCRIPTOR)
  {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  closeRes = close(fd);
  fd = NO_FILE_DESCRIPTOR;

  if (closeRes == -1)
  {
    return BASE_TYPES_OPER_ERROR;
  }

  return BASE_TYPES_OPER_OK;
}

/*
* Returns information about last error or @c NULL, if no error yet occurred.
* @return information about last error
* @return @c NULL if no error yet occurred
*/
//errors_OperError* rpi_spi_iqrf_getLastError(void) {
//  //return errors_getErrorCopy(lastError);
//  return errors_getErrorCopy(0);
//}
