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
#define	IQRF_PGM_SUCCESS              200
#define IQRF_PGM_FLASH_BLOCK_READY    220
#define IQRF_PGM_EEPROM_BLOCK_READY   221
#define	IQRF_PGM_ERROR                222

#define IQRF_PGM_FILE_DATA_READY      0
#define IQRF_PGM_FILE_DATA_ERROR      1
#define IQRF_PGM_END_OF_FILE          2

#define IQRF_SIZE_OF_FLASH_BLOCK      64
#define IQRF_LICENCED_MEMORY_BLOCKS   96
#define IQRF_MAIN_MEMORY_BLOCKS       48

#define IQRF_CFG_MEMORY_BLOCK         (IQRF_LICENCED_MEMORY_BLOCKS - 2)

#define SERIAL_EEPROM_MIN_ADR         0x0200
#define SERIAL_EEPROM_MAX_ADR         0x09FF
#define IQRF_LICENCED_MEM_MIN_ADR     0x2C00
#define IQRF_LICENCED_MEM_MAX_ADR     0x37FF
#define IQRF_CONFIG_MEM_L_ADR         0x37C0
#define IQRF_CONFIG_MEM_H_ADR         0x37D0
#define IQRF_MAIN_MEM_MIN_ADR         0x3A00
#define IQRF_MAIN_MEM_MAX_ADR         0x3FFF
#define PIC16LF1938_EEPROM_MIN        0xf000
#define PIC16LF1938_EEPROM_MAX        0xf0ff

#define SIZE_OF_CODE_LINE_BUFFER      64

/************************************/
/* Private functions predeclaration */
/************************************/
void iqrfPgmMoveOverflowedData(void);
uint8_t iqrfPgmPrepareMemBlock(void);
uint8_t iqrfPgmConvertToNum(uint8_t dataByteHi, uint8_t dataByteLo);
char iqrfReadByteFromFile(void);
uint8_t iqrfPgmReadHEXFileLine(void);
void printDataInHex(unsigned char *data, unsigned int length);
spi_iqrf_SPIStatus tryToWaitForPgmReady(uint32_t timeout);

/************************************/
/* Private variables                */
/************************************/
typedef struct {
    uint32_t  HiAddress;
    uint16_t  Address;
    uint16_t  MemoryBlockNumber;
    uint8_t MemoryBlockProcessState;
    uint8_t DataInBufferReady;
    uint8_t DataOverflow;
    uint8_t MemoryBlock[68];
} PREPARE_MEM_BLOCK;

PREPARE_MEM_BLOCK PrepareMemBlock;

const unsigned long INTERVAL_MS = 10;
uint8_t IqrfPgmCodeLineBuffer[SIZE_OF_CODE_LINE_BUFFER];

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
    int DataSize;
    uint8_t *DataBuffer;

    if ( argc != 2 ) {/* argc should be 2 for correct execution */
        /* We print argv[0] assuming it is the program name */
        printf( "Use *.hex file as input parameter.\n\r");
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

                // initialize variables
                PrepareMemBlock.DataInBufferReady = 0;
                PrepareMemBlock.DataOverflow = 0;
                PrepareMemBlock.MemoryBlockProcessState = 0;

                while (1) {
                    // prepare data to write
                    OpResult = iqrfPgmPrepareMemBlock();
                    // check result of data preparing
                    if (OpResult != IQRF_PGM_FLASH_BLOCK_READY && OpResult != IQRF_PGM_EEPROM_BLOCK_READY)
                        break;  // end programming

                    // write prepared data to TR module
                    while (PrepareMemBlock.MemoryBlockProcessState) {
                        // wait for TR module is ready
                        spiStatus = tryToWaitForPgmReady(2000);
                        // if SPI not ready in 2000 ms, end
                        if (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_PROG) {
                            printf("Waiting for ready state failed.\n\r");
                        } else {
                            if (OpResult == IQRF_PGM_FLASH_BLOCK_READY) {
                                if (PrepareMemBlock.MemoryBlockProcessState == 2) {
                                    DataBuffer = (uint8_t *)&PrepareMemBlock.MemoryBlock[0];
                                    DataSize = 32 + 2;
                                } else {
                                    DataBuffer = (uint8_t *)&PrepareMemBlock.MemoryBlock[34];
                                    DataSize = 32 + 2;
                                }
                            } else {
                                DataBuffer = (uint8_t *)&PrepareMemBlock.MemoryBlock[0];
                                DataSize = PrepareMemBlock.MemoryBlock[1] + 2;
                            }

                            // print prepared data
                            printf("Data to write:\n\r");
                            printDataInHex(DataBuffer, DataSize);
                            printf("Data sent to device.\n\r");

                            // write data to TR module
                            if (OpResult == IQRF_PGM_FLASH_BLOCK_READY)
                                PgmRetCode = spi_iqrf_upload(FLASH_TARGET, DataBuffer, DataSize);
                            else
                                PgmRetCode = spi_iqrf_upload(INTERNAL_EEPROM_TARGET, DataBuffer, DataSize);

                            // check result of write operation
                            if (PgmRetCode != BASE_TYPES_OPER_OK)
                                printf("Data programming failed. Return code %d\n\r", PgmRetCode);
                            else
                                printf("Data programming OK\n\r");
                        }
                        PrepareMemBlock.MemoryBlockProcessState--;
                    }
                }
                // wait for TR module is ready
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
 * Move overflowed data to active block ready to programming
 */
void iqrfPgmMoveOverflowedData(void)
{
    uint16_t MemBlock;
    // move overflowed data to active block
    memcpy((uint8_t *)&PrepareMemBlock.MemoryBlock[34], (uint8_t *)&PrepareMemBlock.MemoryBlock[0], 34);
    // clear block of memory for overfloved data
    memset((uint8_t *)&PrepareMemBlock.MemoryBlock[0], 0, 34);
    // calculate the data block index
    MemBlock = ((uint16_t)PrepareMemBlock.MemoryBlock[35] << 8) | PrepareMemBlock.MemoryBlock[34];
    PrepareMemBlock.MemoryBlockNumber = MemBlock + 0x10;
    MemBlock++;
    PrepareMemBlock.MemoryBlock[0] = MemBlock & 0x00FF;         // write next block index to image
    PrepareMemBlock.MemoryBlock[1] = MemBlock >> 8;
    PrepareMemBlock.DataOverflow = 0;
    // initialize block process counter (block will be written to TR module in 1 write packet)
    PrepareMemBlock.MemoryBlockProcessState = 1;
}

/**
 * Reading and preparing a block of data to be programmed into the TR module
 * @param none
 * @return result of data preparing operation
 */
uint8_t iqrfPgmPrepareMemBlock(void)
{
    uint16_t MemBlock;
    uint8_t DataCounter;
    uint8_t DestinationIndex;
    uint8_t ValidAddress;
    uint8_t OperationResult;
    uint8_t Cnt;

    // initialize memory block for flash programming
    if (!PrepareMemBlock.DataOverflow) {
        for (Cnt=0; Cnt<sizeof(PrepareMemBlock.MemoryBlock); Cnt+=2) {
            PrepareMemBlock.MemoryBlock[Cnt] = 0xFF;
            PrepareMemBlock.MemoryBlock[Cnt+1] = 0x3F;
        }
    }
    PrepareMemBlock.MemoryBlockNumber = 0;

    while(1) {
        // if no data ready in file buffer
        if (!PrepareMemBlock.DataInBufferReady) {
            OperationResult = iqrfPgmReadHEXFileLine();       // read one line from HEX file
            // check result of file reading operation
            if (OperationResult == IQRF_PGM_FILE_DATA_ERROR) {
                return(IQRF_PGM_ERROR);
            } else {
                if (OperationResult == IQRF_PGM_END_OF_FILE) {
                    // if any data are ready to programm to FLASH
                    if (PrepareMemBlock.MemoryBlockNumber) {
                        return(IQRF_PGM_FLASH_BLOCK_READY);
                    } else {
                        if (PrepareMemBlock.DataOverflow) {
                            iqrfPgmMoveOverflowedData();
                            return(IQRF_PGM_FLASH_BLOCK_READY);
                        } else {
                            return(IQRF_PGM_SUCCESS);
                        }
                    }
                }
            }
            PrepareMemBlock.DataInBufferReady = 1;            // set flag, data ready in file buffer
        }

        if (IqrfPgmCodeLineBuffer[3] == 0) {                // data block ready in file buffer
            // read destination address for data in buffer
            PrepareMemBlock.Address = (PrepareMemBlock.HiAddress + ((uint16_t)IqrfPgmCodeLineBuffer[1] << 8) + IqrfPgmCodeLineBuffer[2]) / 2;
            if (PrepareMemBlock.DataOverflow)
                iqrfPgmMoveOverflowedData();
            // data for external serial EEPROM
            if (PrepareMemBlock.Address >= SERIAL_EEPROM_MIN_ADR && PrepareMemBlock.Address <= SERIAL_EEPROM_MAX_ADR) {
                // if image of data block is not initialized
                if (PrepareMemBlock.MemoryBlockNumber == 0) {
                    MemBlock = (PrepareMemBlock.Address - 0x200) / 32;          // calculate data block index
                    memset((uint8_t *)&PrepareMemBlock.MemoryBlock[0], 0, 68);  // clear image of data block
                    PrepareMemBlock.MemoryBlock[34] = MemBlock & 0x00FF;        // write block index to image
                    PrepareMemBlock.MemoryBlock[35] = MemBlock >> 8;
                    MemBlock++;                                                 // next block index
                    PrepareMemBlock.MemoryBlock[0] = MemBlock & 0x00FF;         // write next block index to image
                    PrepareMemBlock.MemoryBlock[1] = MemBlock >> 8;
                    PrepareMemBlock.MemoryBlockNumber = PrepareMemBlock.Address / 32;   // remember actual memory block
                    // initialize block process counter (block will be written to TR module in 1 write packet)
                    PrepareMemBlock.MemoryBlockProcessState = 1;
                }

                MemBlock = PrepareMemBlock.Address / 32;                        // calculate actual memory block
                // calculate offset from start of image, where data to be written
                DestinationIndex = (PrepareMemBlock.Address % 32) + 36;
                DataCounter = IqrfPgmCodeLineBuffer[0] / 2;                     // read number of data bytes in file buffer

                // if data in file buffer are from different memory block, write actual image to TR module
                if (PrepareMemBlock.MemoryBlockNumber != MemBlock)
                    return(IQRF_PGM_FLASH_BLOCK_READY);
                // check if all data are inside the image of data block
                if (DestinationIndex + DataCounter > sizeof(PrepareMemBlock.MemoryBlock))
                    PrepareMemBlock.DataOverflow = 1;
                // copy data from file buffer to image of data block
                for (Cnt=0; Cnt < DataCounter; Cnt++) {
                    PrepareMemBlock.MemoryBlock[DestinationIndex++] = IqrfPgmCodeLineBuffer[2*Cnt+4];
                    if (DestinationIndex == 68)
                        DestinationIndex = 2;
                }
                // if all data are not inside the image of data block
                if (PrepareMemBlock.DataOverflow) {
                    PrepareMemBlock.DataInBufferReady = 0;                      // process next line from HEX file
                    return(IQRF_PGM_FLASH_BLOCK_READY);
                }
            } else {  // check if data in file buffer are for other memory areas
                MemBlock = PrepareMemBlock.Address / 32;                        // calculate actual memory block
                // calculate offset from start of image, where data to be written
                DestinationIndex = (PrepareMemBlock.Address % 32) * 2;
                if (DestinationIndex < 32) DestinationIndex += 2;
                else DestinationIndex += 4;
                DataCounter = IqrfPgmCodeLineBuffer[0];                         // read number of data bytes in file buffer
                ValidAddress = 0;

                // check if data in file buffer are for main FLASH memory area in TR module
                if (PrepareMemBlock.Address >= IQRF_MAIN_MEM_MIN_ADR
                    && PrepareMemBlock.Address <= IQRF_MAIN_MEM_MAX_ADR)
                {
                    ValidAddress = 1;                                           // set flag, data are for FLASH memory area
                    // check if all data are in main memory area
                    if ((PrepareMemBlock.Address + DataCounter/2) > IQRF_MAIN_MEM_MAX_ADR)
                        DataCounter = (IQRF_MAIN_MEM_MAX_ADR - PrepareMemBlock.Address) * 2;
                    // check if all data are inside the image of data block
                    if (DestinationIndex + DataCounter > sizeof(PrepareMemBlock.MemoryBlock))
                        return(IQRF_PGM_ERROR);
                    // if data in file buffer are from different memory block, write actual image to TR module
                    if (PrepareMemBlock.MemoryBlockNumber) {
                        if (PrepareMemBlock.MemoryBlockNumber != MemBlock)
                            return(IQRF_PGM_FLASH_BLOCK_READY);
                    }
                } else {
                    // check if data in file buffer are for licenced FLASH memory area in TR module
                    if (PrepareMemBlock.Address >= IQRF_LICENCED_MEM_MIN_ADR
                        && PrepareMemBlock.Address <= IQRF_LICENCED_MEM_MAX_ADR)
                    {
                        ValidAddress = 1;                                       // set flag, data are for FLASH memory area
                        // check if all data are in licenced memory area
                        if ((PrepareMemBlock.Address + DataCounter/2) > IQRF_LICENCED_MEM_MAX_ADR)
                            DataCounter = (IQRF_LICENCED_MEM_MAX_ADR - PrepareMemBlock.Address) * 2;
                        // check if all data are inside the image of data block
                        if (DestinationIndex + DataCounter > sizeof(PrepareMemBlock.MemoryBlock))
                            return(IQRF_PGM_ERROR);
                        // if data in file buffer are from different memory block, write actual image to TR module
                        if (PrepareMemBlock.MemoryBlockNumber) {
                            if (PrepareMemBlock.MemoryBlockNumber != MemBlock)
                                return(IQRF_PGM_FLASH_BLOCK_READY);
                        }
                    } else {
                        // check if data in file buffer are for internal EEPROM of TR module
                        if (PrepareMemBlock.Address >= PIC16LF1938_EEPROM_MIN
                            && PrepareMemBlock.Address <= PIC16LF1938_EEPROM_MAX)
                        {
                            // if image of data block contains any data, write it to TR module
                            if (PrepareMemBlock.MemoryBlockNumber)
                                return(IQRF_PGM_FLASH_BLOCK_READY);
                            // prepare image of data block for internal EEPROM
                            PrepareMemBlock.MemoryBlock[0] = PrepareMemBlock.Address & 0x00FF;
                            PrepareMemBlock.MemoryBlock[1] = DataCounter / 2;
                            if (PrepareMemBlock.Address + PrepareMemBlock.MemoryBlock[1] > PIC16LF1938_EEPROM_MAX
                                || PrepareMemBlock.MemoryBlock[1] > 32)
                            {
                                return(IQRF_PGM_ERROR);
                            }
                            for (uint8_t Cnt=0; Cnt < PrepareMemBlock.MemoryBlock[1]; Cnt++)
                                PrepareMemBlock.MemoryBlock[Cnt+2] = IqrfPgmCodeLineBuffer[2*Cnt+4];

                            PrepareMemBlock.DataInBufferReady = 0;
                            // initialize block process counter (block will be written to TR module in 1 write packet)
                            PrepareMemBlock.MemoryBlockProcessState = 1;
                            return(IQRF_PGM_EEPROM_BLOCK_READY);
                        }
                    }
                }
                // if destination address is from FLASH memory area
                if (ValidAddress) {
                    // remember actual memory block
                    PrepareMemBlock.MemoryBlockNumber = MemBlock;
                    // initialize block process counter (block will be written to TR module in 2 write packets)
                    PrepareMemBlock.MemoryBlockProcessState = 2;
                    // compute and write destination address of first half of image
                    MemBlock *= 32;
                    PrepareMemBlock.MemoryBlock[0] = MemBlock & 0x00FF;
                    PrepareMemBlock.MemoryBlock[1] = MemBlock >> 8;
                    // compute and write destination address of second half of image
                    MemBlock += 0x0010;
                    PrepareMemBlock.MemoryBlock[34] = MemBlock & 0x00FF;
                    PrepareMemBlock.MemoryBlock[35] = MemBlock >> 8;
                    // copy data from file buffer to image of data block
                    memcpy(&PrepareMemBlock.MemoryBlock[DestinationIndex], &IqrfPgmCodeLineBuffer[4], DataCounter);
                }
            }
        } else {
            if (IqrfPgmCodeLineBuffer[3] == 4)                                 // in file buffer is address info
                PrepareMemBlock.HiAddress = ((uint32_t)IqrfPgmCodeLineBuffer[4] << 24) + ((uint32_t)IqrfPgmCodeLineBuffer[5] << 16);
        }
        PrepareMemBlock.DataInBufferReady = 0;                                 // process next line from HEX file
    }
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
 * Read and process line from HEX file
 * @return Return code (IQRF_PGM_FILE_DATA_READY - iqrf file line ready, IQRF_PGM_FILE_DATA_READY - input file format error, IQRF_PGM_END_OF_FILE - end of file)
 */
uint8_t iqrfPgmReadHEXFileLine(void)
{
    uint8_t Sign;
    uint8_t DataByteHi, DataByteLo;
    uint8_t DataByte;
    uint8_t CodeLineBufferPtr = 0;
    uint8_t CodeLineBufferCrc = 0;

    // find start of line or end of file
    while (((Sign = iqrfReadByteFromFile()) != 0) && (Sign != ':'))
        ; /* void */
    // if end of file
    if (Sign == 0)
        return(IQRF_PGM_END_OF_FILE);

    // read data to end of line and convert if to numbers
    for (;;) {
        // read High nibble
        DataByteHi = tolower(iqrfReadByteFromFile());
        // check end of line
        if (DataByteHi == 0x0A || DataByteHi == 0x0D) {
            if (CodeLineBufferCrc != 0)
                // check line CRC
                return(IQRF_PGM_FILE_DATA_ERROR);
            // stop reading
            return(IQRF_PGM_FILE_DATA_READY);
        }
        // read Low nibble
        DataByteLo = tolower(iqrfReadByteFromFile());
        // convert two ascii to number
        DataByte = iqrfPgmConvertToNum(DataByteHi, DataByteLo);
        // add to Crc
        CodeLineBufferCrc += DataByte;
        // store to line buffer
        IqrfPgmCodeLineBuffer[CodeLineBufferPtr++] = DataByte;
        if (CodeLineBufferPtr >= SIZE_OF_CODE_LINE_BUFFER)
            return (IQRF_PGM_FILE_DATA_ERROR);
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
        if (i != (length - 1))
            printf(" ");
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
