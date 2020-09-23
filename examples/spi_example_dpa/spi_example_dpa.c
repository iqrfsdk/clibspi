/*
* Copyright 2020 MICRORISC s.r.o.
* Copyright 2020 IQRF Tech s.r.o.
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

#include "DPA.h"

/** Defines whole DPA message. */
typedef union {
    uint8_t Buffer[64];
    struct {
        uint16_t NADR;
        uint8_t PNUM;
        uint8_t PCMD;
        uint16_t HWPID;
        uint8_t ResponseCode;
        uint8_t DpaValue;
        TDpaMessage DpaMessage;
    };
    struct {
        uint16_t NADR;
        uint8_t PNUM;
        uint8_t PCMD;
        uint16_t HWPID;
        TDpaMessage DpaMessage;
    } Request;
} T_DPA_PACKET;

/** Defines an alias representing the LED colors. */
typedef enum _LedColor_t {
    Red,
    Green
} LedColor_t;

/************************************/
/* Private constants                */
/************************************/

//#define NANO_SECOND_MULTIPLIER  1000000  // 1 millisecond = 1,000,000 nanoseconds
//const unsigned long INTERVAL_MS = 10 * NANO_SECOND_MULTIPLIER;
const unsigned long INTERVAL_MS = 10;

/** Time interval between DPA messagess controls LEDs. */
const unsigned long TIME_BETWEEN_LEDS_MS = 2000;

/************************************/
/* Private functions predeclaration */
/************************************/

int openCommunication(void);

int closeCommunication(void);

void printErrorAndExit(const char *userMessage, int error, int retValue);

spi_iqrf_SPIStatus tryToWaitForReadyState(uint32_t timeout);

spi_iqrf_SPIStatus tryToWaitForDataReadyState(uint32_t timeout);

int executeDpaCommand(const uint8_t *dpaMessage, int dataLen);

int isConfirmation(T_DPA_PACKET *msg);

void pulseLed(uint16_t address, LedColor_t color);

void printDataInHex(unsigned char *data, unsigned int length);

/************************************/
/* Private variables                */
/** State of the communication.		*/
int communicationOpen = -1;
/** DPA response. */
T_DPA_PACKET dpaResponsePacket;

/**
 * Main entry-point for this application.
 *
 * @return	Exit-code for the process - 0 for success, else an error code.
 */

int main(void)
{
    int operResult;
    int i;
    //struct timespec delayTime = {TIME_BETWEEN_LEDS_S, 0};

    printf("DPA example demo application.\n\r");

    operResult = openCommunication();
    if (operResult) {
        printf("Initialization failed: %d \n", operResult);
        return operResult;
    }

    printf("Communication port has been successfully open ....\n\r");

    for (i = 0; i < 3; ++i) {
        pulseLed(COORDINATOR_ADDRESS, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulseLed(COORDINATOR_ADDRESS, Red);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);

        pulseLed(BROADCAST_ADDRESS, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulseLed(BROADCAST_ADDRESS, Red);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);

        pulseLed(0x01, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulseLed(0x02, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulseLed(0x03, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
    }

    closeCommunication();
    return 0;
}

/**
 * Prints specified user message and specified error description to standard output, cleans up
 * the Rpi_spi_iqrf library, and exits the program with specified return value.
 *
 * @param	userMessage	Message describing the error.
 * @param	error	   	The error identificator.
 * @param	retValue   	The returned value.
 */

void printErrorAndExit(const char *userMessage, int error, int retValue)
{
    printf("%s: %d", userMessage, error);
    closeCommunication();
    exit(retValue);
}

/**
 * Opens SPI communication.
 *
 * @return	0 for success.
 */

int openCommunication(void)
{
    int operResult;

    operResult = spi_iqrf_initDefault();
    if (operResult < 0) {
        printf("Initialization failed: %d \n", operResult);
        return operResult;
    }

    printf("Communication port is open.\n");
    communicationOpen = 1;
    return 0;
}

/**
 * Closes the IQRF communication.
 *
 * @return	0 for success, -1 if communication is already closed.
 */

int closeCommunication(void)
{
    if (communicationOpen < 0) {
        printf("Communication port was not open.\n");
        return -1;
    }

    spi_iqrf_destroy();
    return 0;
}

/**
 * Try to wait for communication ready state in specified timeout (in ms).
 *
 * @param	timeout	Timeout in ms
 *
 * @return	Last read IQRF SPI status.
 */

spi_iqrf_SPIStatus tryToWaitForReadyState(uint32_t timeout)
{
    spi_iqrf_SPIStatus spiStatus = {0, {SPI_IQRF_SPI_DISABLED}};
    int operResult = -1;
    uint32_t elapsedTime = 0;
    //struct timespec sleepValue = {0, INTERVAL_MS};
    uint8_t buffer[64];
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

        if (spiStatus.isDataReady == 1) {
            // reading - only to dispose old data if any
            spi_iqrf_read(buffer, spiStatus.dataReady);
        }
    } while (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_COMM);

    printf("Status: %d x 0x%02x \r\n", repStatCounter, spiStatus.dataNotReadyStatus);
    return spiStatus;
}

/**
 * Try to wait for data ready state in specified timeout (in ms)
 *
 * @param	timeout	Timeout in ms
 *
 * @return	Last read IQRF SPI status.
 */

spi_iqrf_SPIStatus tryToWaitForDataReadyState(uint32_t timeout)
{
    spi_iqrf_SPIStatus spiStatus = {0, {SPI_IQRF_SPI_DISABLED}};
    int operResult = -1;
    uint32_t elapsedTime = 0;
    uint16_t memStatus = 0x8000;
    uint16_t repStatCounter = 1;

    do {
        if (elapsedTime > timeout) {
            printf("Status: %d x 0x%02x \r\n", repStatCounter, spiStatus.dataNotReadyStatus);
            printf("Timeout of waiting on data ready state expired\n");
            return spiStatus;
        }

        //nanosleep(&sleepValue, NULL);
        SLEEP(INTERVAL_MS);
        elapsedTime += 10;

        // getting slave status
        operResult = spi_iqrf_getSPIStatus(&spiStatus);
        if (operResult) {
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
    } while (spiStatus.isDataReady != 1);

    printf("Status: %d x 0x%02x \r\n", repStatCounter, spiStatus.dataNotReadyStatus);
    return spiStatus;
}

/**
 * Pulse with LED selected by @c color on address defined by @c address.
 *
 * @param	address	The address of the destination module.
 * @param	color  	Color of the LED on module.
 */

void pulseLed(uint16_t address, LedColor_t color)
{
    T_DPA_PACKET message;
    message.Request.NADR = address;
    message.Request.HWPID = HWPID_DoNotCheck;
    message.Request.PCMD = CMD_LED_PULSE;
    if (color == Red)
        message.Request.PNUM = PNUM_LEDR;
    else
        message.Request.PNUM = PNUM_LEDG;

    executeDpaCommand((const uint8_t *)&message.Buffer, sizeof(TDpaIFaceHeader));
}

/**
 * Executes the DPA command operation.
 *
 * @param	dpaMessage	DPA message to be executed.
 * @param	dataLen   	Length of the message.
 *
 * @return	0 for success, in case of error, the app is closed.
 */

int executeDpaCommand(const uint8_t *dpaMessage, int dataLen)
{

    spi_iqrf_SPIStatus spiStatus = tryToWaitForReadyState(5000);

    // if SPI not ready in 5000 ms, end
    if (spiStatus.dataNotReadyStatus != SPI_IQRF_SPI_READY_COMM)
        printErrorAndExit("Waiting for ready state failed.\n", 0, 1);

    // sending some data to TR module
    int operResult = spi_iqrf_write((uint8_t *)dpaMessage, dataLen);
    if (operResult)
        printErrorAndExit("Error during data sending", 0, operResult);
    printf("Data successfully sent to SPI device.\n\r");

    int receiveCounter = 0;
    do {
        spiStatus = tryToWaitForDataReadyState(5000);

        if (spiStatus.isDataReady != 1)
            return -1;

        printf("Data len: %i \n", spiStatus.dataReady);
        operResult = spi_iqrf_read(dpaResponsePacket.Buffer, spiStatus.dataReady);
        if (operResult)
            printErrorAndExit("Error during data reading", 0, operResult);
        printf("Data successfully received from IQRF module\n\r");
        printDataInHex(dpaResponsePacket.Buffer, spiStatus.dataReady);
        if (isConfirmation(&dpaResponsePacket)) {
            if (dpaResponsePacket.NADR != BROADCAST_ADDRESS) {
                printf("Confirmation packet received.\n\r");
                continue;
            } else {
                printf("Confirmation for broadcast packet received.\n\r");
            }
        }
        receiveCounter++;

    } while (receiveCounter < 1);

    return 0;
}

/**
 * Determines, if the specified message header is the CONFIRMATION.
 *
 * @param [in,out]	msg	If non-null, the message.
 *
 * @return	1 for Confirmation header.
 * @return	0 for the other headers.
 * @return	-1 for NULL parameter.
 */

int isConfirmation(T_DPA_PACKET *msg)
{
    if (msg == NULL)
        return -1;

    if ((TErrorCodes) msg->ResponseCode == STATUS_CONFIRMATION)
        return 1;
    return 0;
}

/**
 * Prints specified data onto standard output in hex format.
 *
 * @param [in,out]	data	Pointer to data buffer.
 * @param	length			The length of the data.
 */

void printDataInHex(unsigned char *data, unsigned int length)
{
    unsigned int i = 0;

    for (i = 0; i < length; i++) {
        printf("0x%.2x", (int) *data);
        data++;
        if (i != (length - 1)) {
	        printf(" ");
        }
    }
    printf("\n\r");
}
