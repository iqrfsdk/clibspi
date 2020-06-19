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
#include "machines_def.h"
#include "spi_iqrf.h"

typedef enum {
    /// Unknown MCU
    MCU_UNKNOWN = 0,
    /// TR-xxx-11A not supported
    PIC16LF819 = 1,
    /// TR-xxx-21A
    PIC16LF88 = 2,
    /// TR-31B, TR-52B, TR-53B
    PIC16F886 = 3,
    /// TR-52D, TR-54D
    PIC16LF1938 = 4,
} mcuType_t;

typedef enum {
    /// TR-52D
    TR_52D = 0,
    /// TR-58D
    TR_58D_RJ = 1,
    /// TR-72D
    TR_72D = 2,
    /// TR-53D
    TR_53D = 3,
    /// TR-54D
    TR_54D = 8,
    /// TR-55D
    TR_55D = 9,
    /// TR-56D
    TR_56D = 10,
    /// TR-76D
    TR_76D = 11,
} trType_t;

typedef enum {
    /// Not certified by FCC
    FCC_NOT_CERTIFIED = 0,
    /// Certified by FCC
    FCC_CERTIFIED = 1,
} fccCert_t;

/**
 * Decodes and prints TR module identification data
 * @param data Pointer to data buffer with identification data
 * @param size Size of data buffer with identification data
 */
void decodeIdfData(unsigned char *data, unsigned int size);
