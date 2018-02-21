#ifndef SYSFS_GPIO_H
#define SYSFS_GPIO_H

#include "declspec.h"

//#ifdef SYSFS_GPIO_EXPORTS
//#define SYSFS_GPIO_DECLSPEC LIB_EXPORT_SHARED
//#else
//#define SYSFS_GPIO_DECLSPEC LIB_IMPORT_SHARED
//#endif

#define SYSFS_GPIO_DECLSPEC


/** Values that represent GPIO directions. */
typedef enum _gpio_direction {
	///< An enum constant representing GPIO input
	GPIO_DIRECTION_IN = 0,
	///< An enum constant representing GPIO output
	GPIO_DIRECTION_OUT
} gpio_direction;

/**
 * Exports GPIO control from kernel to userspace.
 *
 * @param	gpio	GPIO number.
 *
 * @return	An nonnegative gpio_getValue on success.
 */

SYSFS_GPIO_DECLSPEC int gpio_export(int gpio);

/**
 * Reverse the effect of exporting to userspace.
 *
 * @param	gpio	GPIO number.
 *
 * @return	An nonnegative gpio_getValue on success.
 */

SYSFS_GPIO_DECLSPEC int gpio_unexport(int gpio);

/**
 * Set GPIO direction.
 *
 * @param	gpio	GPIO number.
 * @param	dir 	Requested GPIO gpio_getDirection.
 *
 * @return	An nonnegative gpio_getValue on success.
 */

SYSFS_GPIO_DECLSPEC int gpio_setDirection(int gpio, gpio_direction dir);

/**
 * Reads GPIO direction.
 *
 * @param	gpio	GPIO number.
 *
 * @return	Actual set GPIO gpio_getDirection.
 */

SYSFS_GPIO_DECLSPEC gpio_direction gpio_getDirection(int gpio);

/**
 * Set GPIO output value.
 *
 * @param	gpio	GPIO number.
 * @param	val 	Tha gpio_getValue to be set.
 *
 * @return	An nonnegative gpio_getValue on success.
 */

SYSFS_GPIO_DECLSPEC int gpio_setValue(int gpio, int val);

/**
 * Reads GPIO input value.
 *
 * @param	gpio	The gpio.
 *
 * @return	An int.
 */

SYSFS_GPIO_DECLSPEC int gpio_getValue(int gpio);

/**
 * Sets dirction and gpio_getValue of GPIO.
 *
 * @param	gpio	GPIO number.
 * @param	dir 	GPIO gpio_getDirection.
 * @param	val 	The gpio_getValue for output GPIO.
 *
 * @return	An nonnegative gpio_getValue on success.
 */

SYSFS_GPIO_DECLSPEC int gpio_setup(int gpio, gpio_direction dir, int val);

// cleanup gpio
// SYSFS_GPIO_DECLSPEC void gpio_cleanup(int gpio);
SYSFS_GPIO_DECLSPEC int gpio_cleanup(int gpio);

#endif // SYSFS_GPIO_H
