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

#include "spi_example_idf.h"

/**
 * Main entry-point for this application.
 * @return Exit-code for the process - 0 for success, else an error code.
 */
int main(void) {
    spi_iqrf_config_struct mySpiIqrfConfig;
    uint8_t idfBuffer[32];
    uint8_t idfResult;

    puts("TR module identification demo application.");

    strcpy(mySpiIqrfConfig.spiDev, SPI_IQRF_DEFAULT_SPI_DEVICE);
    mySpiIqrfConfig.powerEnableGpioPin = POWER_ENABLE_GPIO;
    mySpiIqrfConfig.busEnableGpioPin = BUS_ENABLE_GPIO;
    mySpiIqrfConfig.pgmSwitchGpioPin = PGM_SWITCH_GPIO;
    mySpiIqrfConfig.trModuleReset = TR_MODULE_RESET_ENABLE;

    spi_iqrf_initAdvanced(&mySpiIqrfConfig);

    puts("Entering programming mode.");

    if (spi_iqrf_pe() != BASE_TYPES_OPER_OK) {
        puts("Programming mode ERROR.");
        return EXIT_FAILURE;
    }
    puts("Programming mode OK.");

    puts("Reading TR module identification.");
    idfResult = spi_iqrf_get_tr_module_info(idfBuffer, sizeof(idfBuffer));
    if (idfResult == BASE_TYPES_OPER_OK) {
        decodeIdfData(idfBuffer, sizeof(idfBuffer));
    } else {
        printf("\nTR module identification ERROR.\n\n");
    }

    puts("Terminating programming mode.");
    if (spi_iqrf_pt() == BASE_TYPES_OPER_OK) {
        puts("Programming mode termination OK.");
    } else {
        puts("Programming mode termination ERROR.");
    }
    return EXIT_SUCCESS;
}

void decodeIdfData(unsigned char *data, unsigned int size) {
    printf("\nTR module identification data.\n");
    printf("------------------------------\n");
    char tempString[32] = "";
    // decode identification data
    uint32_t moduleId = (uint32_t) data[3] << 24 | (uint32_t) data[2] << 16 | (uint32_t) data[1] << 8 | data[0];
    trType_t moduleType = data[5] >> 4;
    mcuType_t mcuType = data[5] & 0x07;
    fccCert_t fccCertified = (data[5] & 0x08) >> 3;
    uint16_t osVersion = data[4];
    uint16_t osBuild = (uint16_t) data[7] << 8 | data[6];

    // print module Type
    if (moduleId & 0x80000000L) {
        strcat(tempString, "(DC)");
    }
    strcat(tempString, "TR-");

    switch (moduleType) {
        case TR_52D:
            strcat(tempString, "52");
            break;
        case TR_58D_RJ:
            strcat(tempString, "58");
            break;
        case TR_72D:
            strcat(tempString, "72");
            break;
        case TR_53D:
            strcat(tempString, "53");
            break;
        case TR_54D:
            strcat(tempString, "54");
            break;
        case TR_55D:
            strcat(tempString, "55");
            break;
        case TR_56D:
            strcat(tempString, "56");
            break;
        case TR_76D:
            strcat(tempString, "76");
            break;
        default:
            strcat(tempString, "xx");
            break;
    }

    if (mcuType == PIC16LF1938) {
        strcat(tempString, "D");
    }
    strcat(tempString, "x\n");
    printf("Module type:\t\t%s", tempString);

    // print module MCU
    printf("Module MCU:\t\t");
    switch (mcuType) {
        case PIC16LF819:
            printf("PIC16LF819\n");
            break;
        case PIC16LF88:
            printf("PIC16LF88\n");
            break;
        case PIC16F886:
            printf("PIC16F886\n");
            break;
        case PIC16LF1938:
            printf("PIC16LF1938\n");
            break;
        default:
            printf("UNKNOWN\n");
            break;
    }

    // print module MCU
    printf("Module ID:\t\t%.8X\n", moduleId);

    if (osVersion >= 0x43) {
        // print module IBK
        printf("Module IBK:\t\t");
        if (size == 32) {
            for (uint8_t cnt = 16; cnt < 32; cnt++) {
                printf("%.2x ", data[cnt]);
            }
            printf("\n");
        } else {
            printf("---\n");
        }
    }

    // print OS version & build
    printf("OS version:\t\t%X.%02XD (%04X)\n", osVersion >> 4, osVersion & 0xf, osBuild);

    // print module FCC certification
    printf("FCC certification:\t");
    if (fccCertified == FCC_CERTIFIED) {
        printf("YES\n");
    } else {
        printf("NO\n");
    }
    printf("\n");
}
