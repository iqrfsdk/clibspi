#ifndef IQRF_GPIO_H
#define IQRF_GPIO_H

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifndef WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#include "iqrf_debug.h"
#include "sleepWrapper.h"

#define IQRF_GPIO_SYSFS_BASE_PATH "/sys/class/gpio/"
#define IQRF_GPIO_SYSFS_BUFFER_SIZE 64
#define IQRF_GPIO_DIRECTION_BUFFER_SIZE 4
#define IQRF_GPIO_PIN_BUFFER_SIZE 20
#define IQRF_GPIO_VALUE_BUFFER_SIZE 2

static const int64_t IQRF_GPIO_PIN_UNKNOWN = -1;

typedef enum {
	IQRF_GPIO_ACTION_DIRECTION,
	IQRF_GPIO_ACTION_VALUE,
} iqrf_gpio_action_t;

typedef enum {
	IQRF_GPIO_DIRECTION_UNKNOWN = -1,
	IQRF_GPIO_DIRECTION_IN,
	IQRF_GPIO_DIRECTION_OUT,
} iqrf_gpio_direction_t;

typedef enum {
	IQRF_GPIO_ERROR_OK,
	IQRF_GPIO_ERROR_INVALID_PIN,
	IQRF_GPIO_ERROR_OPEN_FAILED,
	IQRF_GPIO_ERROR_WRITE_FAILED,
	IQRF_GPIO_ERROR_NULL_POINTER,
} iqrf_gpio_error_t;

/**
 * Exports the GPIO pin
 * @param pin GPIO pin to export
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_export(int64_t pin);

/**
 * Unexports the GPIO pin
 * @param pin GPIO pin to unexport
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_unexport(int64_t pin);

/**
 * Retrieves the direction for GPIO pin
 * @param pin GPIO pin
 * @param direction GPIO pin direction
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_get_direction(int64_t pin, iqrf_gpio_direction_t *direction);

/**
 * Sets the direction for GPIO pin
 * @param pin GPIO pin
 * @param direction GPIO pin direction
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_set_direction(int64_t pin, iqrf_gpio_direction_t direction);

/**
 * Retrieves the direction for GPIO pin
 * @param pin GPIO pin
 * @param value GPIO pin output value
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_get_value(int64_t pin, bool *value);

/**
 * Sets the direction for GPIO pin
 * @param pin GPIO pin
 * @param value GPIO pin output value
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_set_value(int64_t pin, bool value);

/**
 * Creates sysfs path
 * @param pin GPIO pin
 * @param action GPIO action
 * @param targetPath
 */
void iqrf_gpio_create_sysfs_path(int64_t pin, iqrf_gpio_action_t action, char *targetPath);

/**
 * Initializes a GPIO pin
 * @param pin GPIO pin number
 * @param direction GPIO pin direction
 * @param initialValue Initial output value
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_init(int64_t pin, iqrf_gpio_direction_t direction, bool initialValue);

/**
 * Initializes a GPIO pin as an input
 * @param pin GPIO pin number
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_init_input(int64_t pin);

/**
 * Initializes a GPIO pin as an output
 * @param pin GPIO pin
 * @param initialValue Initial output value
 * @return Execution status
 */
iqrf_gpio_error_t iqrf_gpio_init_output(int64_t pin, bool initialValue);

#endif
