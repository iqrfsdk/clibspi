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

#define UP

#ifdef RPI

/** LED GPIO. */
#define LED_GPIO (22)
/** PGM Switch GPIO. */
#define PGM_SW_GPIO (7)
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

#ifndef SPI_IQRF_SPI_KERNEL_MODULE
	#define SPI_IQRF_SPI_KERNEL_MODULE "spi_bcm2835"
#endif

#endif /* RPI */


#ifdef UP

/** LED GPIO. */
#define LED_GPIO (22)
/** PGM Switch GPIO. */
#define PGM_SW_GPIO (7)
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

#ifndef SPI_IQRF_SPI_KERNEL_MODULE
        #define SPI_IQRF_SPI_KERNEL_MODULE "spi_pxa2xx_platform"
#endif

#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
  	#define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev2.0"
#endif

#endif /* UP */


#ifdef UP2

/** LED GPIO. */
#define LED_GPIO (22)
/** PGM Switch GPIO. */
#define PGM_SW_GPIO (7)
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

#ifndef SPI_IQRF_SPI_KERNEL_MODULE
        #define SPI_IQRF_SPI_KERNEL_MODULE "spi_pxa2xx_platform"
#endif

#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
        #define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev1.0"
#endif

#endif /* UP2 */


#ifdef OPIZ

/** LED GPIO. */
#define LED_GPIO (3)
/** PGM Switch GPIO. */
#define PGM_SW_GPIO (10)
/** Enable GPIO. */
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

#ifndef SPI_IQRF_SPI_KERNEL_MODULE
	#define SPI_IQRF_SPI_KERNEL_MODULE ""
#endif

#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
  	#define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev1.0"
#endif

#endif /* OPI */

#endif /* __MACHINES_DEF_H */
