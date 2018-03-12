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

#ifndef __MACHINES_DEF_H
#define __MACHINES_DEF_H

typedef enum iqrf_pin_driver{
    PIN_GPIO = 0,
    PIN_SPI
} iqrf_pin_driver;

/**
* Switch driver of specified pin to requested driver
*
* @return	Zero on success. Negative value on error.
*/
static int iqrf_switch_pin_driver(int pin, iqrf_pin_driver driver);

#define RPI

#ifdef RPI

/** LED GPIO. */
#define LED_GPIO (22)
/** Button GPIO. */
#define BUTTON_GPIO (7)
/** Enable GPIO. */
#define ENABLE_GPIO (23)
/** SPI CE GPIO. */
#define CE0_GPIO (8)
/** SPI MISO GPIO */
#define MISO_GPIO (9)
/** SPI MOSI GPIO */
#define MOSI_GPIO (10)
/** SPI SCLK GPIO */
#define SCLK_GPIO (11)
/** TR IO1 GPIO. */
#define IO1_GPIO (24)
/** TR IO2 GPIO. */
#define IO2_GPIO (25)

static int iqrf_switch_pin_driver(int pin, iqrf_pin_driver driver) {
    return 0;
}

#endif /* RPI */

#ifdef OPIZ

/** LED GPIO. */
#define LED_GPIO (3)
/** Button GPIO. */
#define BUTTON_GPIO (10)
/** Reset GPIO. */
#define ENABLE_GPIO (19)
/** SPI CE GPIO. */
#define CE0_GPIO (13)
/** SPI MISO GPIO */
#define MISO_GPIO (16)
/** SPI MOSI GPIO */
#define MOSI_GPIO (15)
/** SPI SCLK GPIO */
#define SCLK_GPIO (14)
/** TR IO1 GPIO. */
#define IO1_GPIO (18)
/** TR IO2 GPIO. */
#define IO2_GPIO (2)

static int iqrf_switch_pin_driver(int pin, iqrf_pin_driver driver) {
    return 0;
}

#endif /* OPI */

#ifdef UNIPI

/** LED GPIO - NOT USED. */
#define LED_GPIO (0)
/** Button GPIO - NOT USED. */
#define BUTTON_GPIO (0)
/** Reset GPIO. */
#define ENABLE_GPIO (18)
/** SPI CE GPIO. */
#define CE0_GPIO (8)
/** SPI MISO GPIO */
#define MISO_GPIO (9)
/** SPI MOSI GPIO */
#define MOSI_GPIO (10)
/** SPI SCLK GPIO */
#define SCLK_GPIO (11)
/** TR IO1 GPIO. */
#define IO1_GPIO (17)
/** TR IO2 GPIO. */
#define IO2_GPIO (4)

static int iqrf_switch_pin_driver(int pin, iqrf_pin_driver driver) {
    return 0;
}

#endif /* UNIPI */

#endif /* __MACHINES_DEF_H */ 
