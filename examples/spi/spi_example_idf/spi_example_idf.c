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
void decodeIdfData(unsigned char *data);

/************************************/
/* Private variables                */
/** SPI IQRF configuration structure */
spi_iqrf_config_struct mySpiIqrfConfig;

/**
 * Main entry-point for this application.
 *
 * @return	Exit-code for the process - 0 for success, else an error code.
 */

int main(void)
{
    uint8_t idfBuffer[16];
    uint8_t idfResult;

    printf("TR module identification demo application.\n\r");

    strcpy (mySpiIqrfConfig.spiDev, SPI_IQRF_DEFAULT_SPI_DEVICE);
    mySpiIqrfConfig.enableGpioPin = ENABLE_GPIO;
    mySpiIqrfConfig.spiMasterEnGpioPin = SPI_MASTER_EN_GPIO;
    mySpiIqrfConfig.spiPgmSwGpioPin = PGM_SW_GPIO;

    spi_iqrf_initAdvanced(&mySpiIqrfConfig);

    printf("Entering programming mode.\n\r");

    if (spi_iqrf_pe() == BASE_TYPES_OPER_OK) {
      printf("Programming mode OK.\n\r");

      printf("Reading TR module identification.\n\r");
      idfResult = spi_iqrf_get_tr_module_info(idfBuffer, sizeof(idfBuffer));
      if (idfResult == BASE_TYPES_OPER_OK) {
        decodeIdfData(idfBuffer);
      } else {
        printf("\n\rTR module identification ERROR.\n\r\n\r");
      }

      printf("Terminating programming mode.\n\r");
      if (spi_iqrf_pt() == BASE_TYPES_OPER_OK) {
        printf("Programming mode termination OK.\n\r");
      } else {
        printf("Programming mode termination ERROR.\n\r");
      }

    } else {
      printf("Programming mode ERROR.\n\r");
    }

    return 0;
}

/**
 * Decode and print TR module identification data
 *
 * @param [in]	data	Pointer to data buffer with identification data
 */
void decodeIdfData(unsigned char *data)
{
  // MCU type of TR module
  #define MCU_UNKNOWN                   0
  #define PIC16LF819                    1     // TR-xxx-11A not supported
  #define PIC16LF88                     2     // TR-xxx-21A
  #define PIC16F886                     3     // TR-31B, TR-52B, TR-53B
  #define PIC16LF1938                   4     // TR-52D, TR-54D

  // TR module types
  #define TR_52D                        0
  #define TR_58D_RJ                     1
  #define TR_72D                        2
  #define TR_53D                        3
  #define TR_54D                        8
  #define TR_55D                        9
  #define TR_56D                        10
  #define TR_76D                        11

  // FCC cerificate
  #define FCC_NOT_CERTIFIED             0
  #define FCC_CERTIFIED                 1

  uint32_t moduleId;
  uint16_t osBuild;
  uint8_t osVersionMajor, osVersionMinor;
  uint8_t moduleType;
  uint8_t mcuType;
  uint8_t fccCerificate;
  uint8_t tempString[32];
  uint8_t tempStringPtr = 0;

  printf("\n\rTR module identification data.\n\r");
  printf("------------------------------\n\r");

  // decode identification data
  moduleId = (uint32_t)data[3] << 24 | (uint32_t)data[2] << 16 | (uint32_t)data[1] << 8 | data[0];
  moduleType = data[5] >> 4;
  mcuType = data[5] & 0x07;
  fccCerificate = (data[5] & 0x08) >> 3;
  osVersionMajor = data[4] / 16;
  osVersionMinor = data[4] % 16;
  osBuild = (uint16_t)data[7] << 8 | data[6];

  // print module Type
  if (moduleId & 0x80000000L){
    tempString[tempStringPtr++] = 'D';
    tempString[tempStringPtr++] = 'C';
  }
  tempString[tempStringPtr++] = 'T';
  tempString[tempStringPtr++] = 'R';
  tempString[tempStringPtr++] = '-';
  tempString[tempStringPtr++] = '5';

  switch(moduleType){
    case TR_52D: tempString[tempStringPtr++] = '2'; break;
    case TR_58D_RJ: tempString[tempStringPtr++] = '8'; break;
    case TR_72D: tempString[tempStringPtr-1] = '7'; tempString[tempStringPtr++] = '2'; break;
    case TR_53D: tempString[tempStringPtr++] = '3'; break;
    case TR_54D: tempString[tempStringPtr++] = '4'; break;
    case TR_55D: tempString[tempStringPtr++] = '5'; break;
    case TR_56D: tempString[tempStringPtr++] = '6'; break;
    case TR_76D: tempString[tempStringPtr-1] = '7'; tempString[tempStringPtr++] = '6'; break;
    default : tempString[tempStringPtr++] = 'x'; break;
  }

  if(mcuType == PIC16LF1938) tempString[tempStringPtr++]='D';
  tempString[tempStringPtr++]='x';
  tempString[tempStringPtr++] = 0;
  strcat(tempString, "\n\r");
  printf("Module type:   ");
  printf(tempString);

  // print module MCU
  printf("Module MCU:    ");
  switch (mcuType) {
    case PIC16LF819: printf("PIC16LF819\n\r"); break;
    case PIC16LF88: printf("PIC16LF88\n\r"); break;
    case PIC16F886: printf("PIC16F886\n\r"); break;
    case PIC16LF1938: printf("PIC16LF1938\n\r"); break;
    default: printf("UNKNOWN\n\r"); break;
  }

  // print module MCU
  printf("Module ID:     %.8X\n\r", moduleId);

  // print OS version & build
  printf("OS version:   %2X.%02XD (0x%04x)\n\r", osVersionMajor, osVersionMinor, osBuild);

  // print module FCC certification
  printf("Module FCC certification: ");
  if (fccCerificate == FCC_CERTIFIED) printf("YES\n\r");
  else printf("NO\n\r");

  printf("\n\r");
}
