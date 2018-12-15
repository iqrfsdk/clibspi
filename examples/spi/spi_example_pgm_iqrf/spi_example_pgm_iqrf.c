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
#define IQRF_PGM_FILE_DATA_READY      0
#define IQRF_PGM_FILE_DATA_ERROR      1
#define IQRF_PGM_END_OF_FILE          2

/************************************/
/* Private functions predeclaration */
/************************************/
uint8_t iqrfPgmConvertToNum(uint8_t dataByteHi, uint8_t dataByteLo);
uint8_t iqrfPgmReadIQRFFileLine(void);
void printDataInHex(unsigned char *data, unsigned int length);
char iqrfReadByteFromFile(void);
spi_iqrf_SPIStatus tryToWaitForPgmReady(uint32_t timeout);

/************************************/
/* Private variables                */
/************************************/
const unsigned long INTERVAL_MS = 10;
uint8_t IqrfPgmCodeLineBuffer[64];

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
int main ( int argc, char *argv[] )
{
    int OpResult;
    int PgmRetCode;

    if ( argc != 2 ) {/* argc should be 2 for correct execution */
        /* We print argv[0] assuming it is the program name */
        printf( "Use *.iqrf file as input parameter.\n\r");
    } else {
        // We assume argv[1] is a filename to open
        file = fopen( argv[1], "r" );

        /* fopen returns 0, the NULL pointer, on failure */
        if ( file == 0 ) {
            printf( "Could not open file\n\r" );
        } else {
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

                while ((OpResult = iqrfPgmReadIQRFFileLine()) == IQRF_PGM_FILE_DATA_READY) {
                    spiStatus = tryToWaitForPgmReady(2000);
                    // if SPI not ready in 5000 ms, end
                    if (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG) {
                        printf("Waiting for ready state failed.\n\r");
                    } else {
                        printf("Data to write:\n\r");
                        printDataInHex(IqrfPgmCodeLineBuffer, 20);
                        printf("Data sent to device.\n\r");
                        // write data to TR module
                        PgmRetCode = spi_iqrf_upload(SPECIAL_TARGET, IqrfPgmCodeLineBuffer, 20);
                        if (PgmRetCode != BASE_TYPES_OPER_OK)
                            printf("Data programming failed. Return code %d\n\r", PgmRetCode);
                        else
                            printf("Data programming OK\n\r");
                    }
                }

                spiStatus = tryToWaitForPgmReady(2000);

                // terminate programming mode
                printf("Terminating programming mode.\n\r");
                if (spi_iqrf_pt() == BASE_TYPES_OPER_OK)
                    printf("Programming mode termination OK.\n\r");
                else
                    printf("Programming mode termination ERROR.\n\r");
            } else {
                printf("Programming mode ERROR.\n\r");
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

    do {
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
        } else {
            if (memStatus != spiStatus.dataNotReadyStatus) {
                if (memStatus != 0x8000)
                    printf("Status: %d x 0x%02x \r\n", repStatCounter, memStatus);
                memStatus = spiStatus.dataNotReadyStatus;
                repStatCounter = 1;
            } else {
                repStatCounter++;
            }
        }

        if (spiStatus.isDataReady == 1)
            // reading - only to dispose old data if any
            spi_iqrf_read(buffer, spiStatus.dataReady);
    } while (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG);

    printf("Status: %d x 0x%02x \r\n", repStatCounter, spiStatus.dataNotReadyStatus);
    return spiStatus;
}


/**
 * Convert two ASCII chars to number
 * @param dataByteHi High nibble in ASCII
 * @param dataByteLo Low nibble in ASCII
 * @return Number
 */
uint8_t iqrfPgmConvertToNum(uint8_t dataByteHi, uint8_t dataByteLo)
{
    uint8_t result = 0;

    /* convert High nibble */
    if (dataByteHi >= '0' && dataByteHi <= '9')
        result = (dataByteHi - '0') << 4;
    else if (dataByteHi >= 'a' && dataByteHi <= 'f')
        result = (dataByteHi - 87) << 4;

    /* convert Low nibble */
    if (dataByteLo >= '0' && dataByteLo <= '9')
        result |= (dataByteLo - '0');
    else if (dataByteLo >= 'a' && dataByteLo <= 'f')
        result |= (dataByteLo - 87);

    return(result);
}

/**
 * Read one char from input file
 * @return 0 - END OF FILE or char)
 */
char iqrfReadByteFromFile(void)
{
    int ReadChar;

    ReadChar = fgetc( file );

    if (ReadChar == EOF)
        return 0;

    return ReadChar;
}

/**
 * Read and process line from plugin file
 * @return Return code (IQRF_PGM_FILE_DATA_READY - iqrf file line ready, IQRF_PGM_FILE_DATA_READY - input file format error, IQRF_PGM_END_OF_FILE - end of file)
 */
uint8_t iqrfPgmReadIQRFFileLine(void)
{
    uint8_t FirstChar;
    uint8_t SecondChar;
    uint8_t CodeLineBufferPtr = 0;

repeat_read:
    // read one char from file
    FirstChar = tolower(iqrfReadByteFromFile());

    // read one char from file
    if (FirstChar == '#') {
        // read data to end of line
        while (((FirstChar = iqrfReadByteFromFile()) != 0) && (FirstChar != 0x0D))
            ; /* void */
    }

    // if end of line
    if (FirstChar == 0x0D) {
        // read second code 0x0A
        iqrfReadByteFromFile();
        if (CodeLineBufferPtr == 0)
            // read another line
            goto repeat_read;
        if (CodeLineBufferPtr == 20)
            // line with data readed successfully
            return(IQRF_PGM_FILE_DATA_READY);
        else
            // wrong file format (error)
            return(IQRF_PGM_FILE_DATA_ERROR);
    }

    // if end of file
    if (FirstChar == 0)
        return(IQRF_PGM_END_OF_FILE);

    // read second character from code file
    SecondChar = tolower(iqrfReadByteFromFile());
    if (CodeLineBufferPtr >= 20)
        return(IQRF_PGM_FILE_DATA_ERROR);

    // convert chars to number and store to buffer
    IqrfPgmCodeLineBuffer[CodeLineBufferPtr++] = iqrfPgmConvertToNum(FirstChar, SecondChar);
    // read next data
    goto repeat_read;
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
        if (i != (length - 1))
            printf(" ");
    }
    printf("\n\r");
}
