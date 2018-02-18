#ifndef __MACHINES_DEF_H
#define __MACHINES_DEF_H

#define RPI

#ifdef RPI
/** LED GPIO. */
#define LED_GPIO (22)
/** Button GPIO. */
#define BUTTON_GPIO (7)
/** Reset GPIO. */
#define RESET_GPIO (23)
/** SPI CE GPIO. */
#define RPIIO_PIN_CE0 (8)
/** TR IO1 GPIO. */
#define IO1_GPIO (24)
/** TR IO2 GPIO. */
#define IO2_GPIO (25)
#endif /* RPI */

#ifdef OPIZ
/** LED GPIO. */
#define LED_GPIO (3)
/** Button GPIO. */
#define BUTTON_GPIO (10)
/** Reset GPIO. */
#define RESET_GPIO (19)
/** SPI CE GPIO. */
#define RPIIO_PIN_CE0 (13)
/** TR IO1 GPIO. */
#define IO1_GPIO (18)
/** TR IO2 GPIO. */
#define IO2_GPIO (2)
#endif /* OPI */

#ifdef UNIPI
/** LED GPIO - NOT USED. */
#define LED_GPIO (0)
/** Button GPIO - NOT USED. */
#define BUTTON_GPIO (0)
/** Reset GPIO. */
#define RESET_GPIO (18)
/** SPI CE GPIO. */
#define RPIIO_PIN_CE0 (8)
/** TR IO1 GPIO. */
#define IO1_GPIO (17)
/** TR IO2 GPIO. */
#define IO2_GPIO (4)
#endif /* UNIPI */

#endif /* __MACHINES_DEF_H */ 
