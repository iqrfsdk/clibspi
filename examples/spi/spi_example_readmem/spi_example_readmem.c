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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
//#include <linux/types.h>
#include "machines_def.h"
#include "spi_iqrf.h"
#include "sleepWrapper.h"

/************************************/
/* Private constants                */
/************************************/

/************************************/
/* Private functions predeclaration */
/************************************/
void printDataInHex(unsigned char *data, unsigned int length);
spi_iqrf_SPIStatus tryToWaitForPgmReady(uint32_t timeout);

/************************************/
/* Private variables                */
/************************************/
const unsigned long INTERVAL_MS = 10;

/** SPI IQRF status */
spi_iqrf_SPIStatus spiStatus;

/** SPI IQRF configuration structure */
spi_iqrf_config_struct mySpiIqrfConfig;

/**
 * Main entry-point for this application.
 *
 * @return	Exit-code for the process - 0 for success, else an error code.
 */
int main ( int argc, char *argv[] )
{
  int OpResult;
  uint8_t Cnt;
  uint8_t ReadDataBuffer[32];
  uint8_t WriteDataBuffer[8];
  uint8_t WriteDataLen;
  uint8_t Target;
  uint16_t  MemAddress;
  uint16_t  TempData;

  if ( argc != 2 ) {/* argc should be 2 for correct execution */
    /* We print argv[0] assuming it is the program name */
    printf( "Use -eeprom or -eeeprom of -flash as input parameter.\n\r");
    return(-1);
  }

  if (strcmp(argv[1], "-eeprom") == 0) Target = INTERNAL_EEPROM_TARGET;
  else {
    if (strcmp(argv[1], "-eeeprom") == 0) Target = EXTERNAL_EEPROM_TARGET;
    else {
      if (strcmp(argv[1], "-flash") == 0) Target = FLASH_TARGET;
      else {
        printf( "Wrong parameter.\n\r");
        return(-2);
      }
    }
  }

  // initialize clibspi
  strcpy (mySpiIqrfConfig.spiDev, SPI_IQRF_DEFAULT_SPI_DEVICE);
  mySpiIqrfConfig.enableGpioPin = ENABLE_GPIO;
  mySpiIqrfConfig.spiMasterEnGpioPin = SPI_MASTER_EN_GPIO;
  mySpiIqrfConfig.spiPgmSwGpioPin = PGM_SW_GPIO;

  spi_iqrf_initAdvanced(&mySpiIqrfConfig);

  printf("Entering programming mode.\n\r");
  // enter programming mode
  if (spi_iqrf_pe() == BASE_TYPES_OPER_OK) {
    printf("Programming mode OK.\n\r");

    if (Target == FLASH_TARGET) MemAddress = 0x3A00;    // start address for FLASH
    else MemAddress = 0x0000;                           // start address for internal / external EEPROM

    for (Cnt = 0; Cnt < 8; Cnt++) {
      // wait for TR module is ready
      spiStatus = tryToWaitForPgmReady(2000);
      // if SPI not ready in 2000 ms, end
      if (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG) {
        printf("Waiting for ready state failed.\n\r");
      }
      else {
        switch (Target){
          case INTERNAL_EEPROM_TARGET:
            WriteDataBuffer[0] = MemAddress & 0x00FF;
            WriteDataLen = 1;
            printf("Reading 32 bytes of data from internal EEPROM - Address 0x%02x\n\r", MemAddress);
            break;

          case EXTERNAL_EEPROM_TARGET:
            TempData = (MemAddress / 0x20) + 0x400;
            WriteDataBuffer[0] = TempData & 0x00FF;
            WriteDataBuffer[1] = TempData >> 8;
            WriteDataLen = 2;
            printf("Reading 32 bytes of data from external EEPROM - Address 0x%04x\n\r", MemAddress);
            break;

          case FLASH_TARGET:
            WriteDataBuffer[0] = MemAddress & 0x00FF;
            WriteDataBuffer[1] = MemAddress >> 8;
            WriteDataLen = 2;
            printf("Reading 32 bytes of verify data from FLASH - Address 0x%04x\n\r", MemAddress);
            break;
        }

        OpResult = spi_iqrf_download(Target, WriteDataBuffer, WriteDataLen, ReadDataBuffer, 32);

        // check result of write operation
        if (OpResult != BASE_TYPES_OPER_OK) {
          printf("Data reading error. Return code %d\n\r", OpResult);
        }
        else {
          // print readed data
          printf("Data:\n\r");
          printDataInHex(ReadDataBuffer, 32);
        }
        // address for next memory block
        MemAddress += 32;
      }
    }

    // wait for TR module is ready
    spiStatus = tryToWaitForPgmReady(2000);

    // terminate programming mode
    printf("Terminating programming mode.\n\r");
    if (spi_iqrf_pt() == BASE_TYPES_OPER_OK) {
      printf("Programming mode termination OK.\n\r");
    }
    else {
      printf("Programming mode termination ERROR.\n\r");
    }
  }
  else {
    printf("Programming mode ERROR.\n\r");
  }
}

/**
 * Prints specified data onto standard output in hex format.
 *
 * @param [in,out]	data	Pointer to data buffer.
 * @param	length			The length of the data.
 */

void printDataInHex(unsigned char *data, unsigned int length)
{
  int i = 0;

  for (i = 0; i < length; i++) {
    printf("0x%.2x", (int) *data);
    data++;
    if (i != (length - 1)) {
      printf(" ");
    }
  }
  printf("\n\r");
}

/**
 * Try to wait for communication ready state in specified timeout (in ms).
 *
 * @param	timeout	Timeout in ms
 *
 * @return	Last read IQRF SPI status.
 */

spi_iqrf_SPIStatus tryToWaitForPgmReady(uint32_t timeout)
{
  spi_iqrf_SPIStatus spiStatus = {0, SPI_IQRF_SPI_DISABLED};
  int operResult = -1;
  uint32_t elapsedTime = 0;
  //struct timespec sleepValue = {0, INTERVAL_MS};
  uint8_t buffer[64];
  unsigned int dataLen = 0;
  uint16_t memStatus = 0x8000;
  uint16_t repStatCounter = 1;

  do
  {
    if (elapsedTime > timeout) {
      printf("Status: %d x 0x%02x \r\n", repStatCounter, spiStatus.dataNotReadyStatus);
      printf("Timeout of waiting on ready state expired\n");
      return spiStatus;
    }

    //nanosleep(&sleepValue, NULL);
    SLEEP(INTERVAL_MS);
    elapsedTime += 10;

    // getting slave status
    operResult = spi_iqrf_getSPIStatus(&spiStatus);
    if (operResult < 0) {
      printf("Failed to get SPI status: %d \n", operResult);
    }
    else {
      if (memStatus != spiStatus.dataNotReadyStatus) {
        if (memStatus != 0x8000) {
          printf("Status: %d x 0x%02x \r\n", repStatCounter, memStatus);
        }
        memStatus = spiStatus.dataNotReadyStatus;
        repStatCounter = 1;
      }
      else repStatCounter++;
    }

    if (spiStatus.isDataReady == 1) {
      // reading - only to dispose old data if any
      spi_iqrf_read(buffer, spiStatus.dataReady);
    }
  } while (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG);
  printf("Status: %d x 0x%02x \r\n", repStatCounter, spiStatus.dataNotReadyStatus);
  return spiStatus;
}
