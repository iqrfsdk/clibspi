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
#define IQRF_CONFIG_MEM_L_ADR     0x37C0
#define IQRF_CONFIG_MEM_H_ADR     0x37D0

/************************************/
/* Private functions predeclaration */
/************************************/
void printDataInHex(unsigned char *data, unsigned int length);
spi_iqrf_SPIStatus tryToWaitForPgmReady(uint32_t timeout);

/************************************/
/* Private variables                */
/************************************/
const unsigned long INTERVAL_MS = 10;
uint8_t FirstHalfOfCfg[34];
uint8_t SecondHalfOfCfg[34];
uint8_t RfPgmCfg;

/** SPI IQRF status */
spi_iqrf_SPIStatus spiStatus;

/** SPI IQRF configuration structure */
spi_iqrf_config_struct mySpiIqrfConfig;

FILE *file = NULL;

/**
 * Main entry-point for this application.
 *
 * @return	Exit-code for the process - 0 for success, else an error code.
 */
int main ( int argc, char *argv[] ) {

  int PgmRetCode;
  int Cnt, DataSize;
  uint32_t CfgFileSize;
  uint8_t *DataBuffer = NULL;

  if ( argc != 2 ) {/* argc should be 2 for correct execution */
    /* We print argv[0] assuming it is the program name */
    printf( "Use *.trcnfg file as input parameter.\n\r");
  }
  else {
    // We assume argv[1] is a filename to open
    file = fopen( argv[1], "r" );

    /* fopen returns 0, the NULL pointer, on failure */
    if ( file == 0 ) {
      printf( "Could not open file\n\r" );
    }
    else {
      // check configuration file size
      fseek(file, 0, SEEK_END);
      CfgFileSize = ftell(file);
      fseek(file, 0, SEEK_SET);
      if (CfgFileSize < 33) {
        printf( "Wrong format of *.trcnfg file\n\r" );
      }
      else {
        // prepare configuration data
        FirstHalfOfCfg[0] = IQRF_CONFIG_MEM_L_ADR & 0x00FF;
        FirstHalfOfCfg[1] = IQRF_CONFIG_MEM_L_ADR >> 8;
        for(Cnt=0; Cnt<16; Cnt++){
          FirstHalfOfCfg[Cnt*2 + 2] = fgetc(file);
          FirstHalfOfCfg[Cnt*2 + 3] = 0x34;
        }

        SecondHalfOfCfg[0] = IQRF_CONFIG_MEM_H_ADR & 0x00FF;
        SecondHalfOfCfg[1] = IQRF_CONFIG_MEM_H_ADR >> 8;;
        for(Cnt=0; Cnt<16; Cnt++){
          SecondHalfOfCfg[Cnt*2 + 2] = fgetc(file);
          SecondHalfOfCfg[Cnt*2 + 3] = 0x34;
        }

        RfPgmCfg = fgetc(file);

        // initialize clibspi
        strcpy (mySpiIqrfConfig.spiDev, SPI_IQRF_DEFAULT_SPI_DEVICE);
        mySpiIqrfConfig.powerEnableGpioPin = POWER_ENABLE_GPIO;
        mySpiIqrfConfig.busEnableGpioPin = BUS_ENABLE_GPIO;
        mySpiIqrfConfig.pgmSwitchGpioPin = PGM_SWITCH_GPIO;

        spi_iqrf_initAdvanced(&mySpiIqrfConfig);

        printf("Entering programming mode.\n\r");
        // enter programming mode
        if (spi_iqrf_pe() == BASE_TYPES_OPER_OK) {
          printf("Programming mode OK.\n\r");

          for (Cnt=0; Cnt<3; Cnt++){
            switch(Cnt){
              case 0:{
                DataBuffer = FirstHalfOfCfg;
                DataSize = 34;
              }
              break;

              case 1:{
                DataBuffer = SecondHalfOfCfg;
                DataSize = 34;
              }
              break;

              case 2:{
                DataBuffer = &RfPgmCfg;
                DataSize = 1;
              }
              break;
            }

            spiStatus = tryToWaitForPgmReady(2000);
            // if SPI not ready in 5000 ms, end
            if (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG) {
              printf("Waiting for ready state failed.\n\r");
            }
            else {

              printf("Data to write:\n\r");
              printDataInHex(DataBuffer, DataSize);
              printf("Data sent to device.\n\r");

              // write data to TR module
              if (Cnt < 2){
                PgmRetCode = spi_iqrf_upload(FLASH_TARGET, DataBuffer, DataSize);
              }
              else {
                PgmRetCode = spi_iqrf_upload(RFPMG_TARGET, DataBuffer, DataSize);
              }

              if (PgmRetCode != BASE_TYPES_OPER_OK) {
                printf("Data programming failed. Return code %d\n\r", PgmRetCode);
              }
              else {
                printf("Data programming OK\n\r");
              }
            }
          }

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
    }
  }
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
