#ifndef __MACHINES_DEF_H
#define __MACHINES_DEF_H

#define RPI

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

#ifdef RPI
/** LED GPIO. */
#define LED_GPIO (22)
/** Button GPIO. */
#define BUTTON_GPIO (7)
/** Reset GPIO. */
#define RESET_GPIO (23)
/** SPI CE GPIO. */
#define RPIIO_PIN_CE0 (8)
/** SPI MISO GPIO */
#define MISO_GPIO (9)
/** SPI MOSI GPIO */
#define MOSI_GPIO (10)
/** SPI SCLK GPIO */
#define SCLK_GPIO (11)

static int iqrf_switch_pin_driver(int pin, iqrf_pin_driver driver) {
    return 0;
}

#endif /* RPI */

#endif /* __MACHINES_DEF_H */ 
