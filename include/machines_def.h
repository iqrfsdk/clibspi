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

#endif /* RPI */

#endif /* __MACHINES_DEF_H */ 
