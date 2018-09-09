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

#define RPI

#ifdef RPI

/** PGM Switch GPIO. */
#define PGM_SW_GPIO (22)
/** SPI master enable GPIO. */
#define SPI_MASTER_EN_GPIO (7)
/** Enable GPIO. */
#define ENABLE_GPIO (23)

#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
	#define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev0.0"
#endif

#endif /* RPI */


#ifdef UP

/** PGM Switch GPIO. */
#define PGM_SW_GPIO (22)
/** SPI master enable GPIO. */
#define SPI_MASTER_EN_GPIO (7)
/** Enable GPIO. */
#define ENABLE_GPIO (23)

#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
  	#define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev2.0"
#endif

#endif /* UP */


#ifdef UP2

/** PGM Switch GPIO. */
#define PGM_SW_GPIO (22)
/** SPI master enable GPIO. */
#define SPI_MASTER_EN_GPIO (7)
/** Enable GPIO. */
#define ENABLE_GPIO (23)

#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
        #define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev1.0"
#endif

#endif /* UP2 */


#ifdef OPIZ

/** PGM Switch GPIO. */
#define PGM_SW_GPIO (3)
/** SPI master enable GPIO. */
#define SPI_MASTER_EN_GPIO (10)
/** Enable GPIO. */
#define ENABLE_GPIO (19)

#ifndef SPI_IQRF_DEFAULT_SPI_DEVICE
  	#define SPI_IQRF_DEFAULT_SPI_DEVICE "/dev/spidev1.0"
#endif

#endif /* OPI */

#endif /* __MACHINES_DEF_H */
