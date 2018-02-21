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
#define RESET_GPIO (19)
/** SPI CE GPIO. */
#define RPIIO_PIN_CE0 (13)
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
#define RESET_GPIO (18)
/** SPI CE GPIO. */
#define RPIIO_PIN_CE0 (8)
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
