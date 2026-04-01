#include "iqrf_gpio.h"

#ifdef WIN32
#define snprintf _snprintf
#endif

static const char* IQRF_GPIO_ACTION_DIRECTION_STR = "direction";
static const char* IQRF_GPIO_ACTION_VALUE_STR = "value";
static const char* IQRF_GPIO_DIRECTION_IN_STR = "in";
static const char* IQRF_GPIO_DIRECTION_OUT_STR = "out";

void iqrf_gpio_create_sysfs_path(int64_t pin, iqrf_gpio_action_t action, char *targetPath) {
	const char *actionString = action == IQRF_GPIO_ACTION_DIRECTION ? IQRF_GPIO_ACTION_DIRECTION_STR : IQRF_GPIO_ACTION_VALUE_STR;
	snprintf(targetPath, IQRF_GPIO_SYSFS_BUFFER_SIZE, IQRF_GPIO_SYSFS_BASE_PATH"gpio%"PRId64"/%s", pin, actionString);
}

iqrf_gpio_error_t iqrf_gpio_export(int64_t pin) {
#ifdef WIN32
	return IQRF_GPIO_ERROR_OK;
#else
	if (pin < 0) {
		IQRF_DEBUG_PRINTF("Invalid GPIO pin number: %"PRId64, pin);
		return IQRF_GPIO_ERROR_INVALID_PIN;
	}
	char *path = IQRF_GPIO_SYSFS_BASE_PATH"export";
	int fd = open(path, O_WRONLY);
	if (fd == -1) {
		IQRF_DEBUG_PRINTF("Unable to open path \"%s\". Reason: %s", path, strerror(errno));
		return IQRF_GPIO_ERROR_OPEN_FAILED;
	}
	char buffer[IQRF_GPIO_PIN_BUFFER_SIZE] = "";
	snprintf(buffer, IQRF_GPIO_PIN_BUFFER_SIZE, "%"PRId64, pin);
	ssize_t writtenSize = write(fd, buffer, IQRF_GPIO_PIN_BUFFER_SIZE);
	if (writtenSize == -1) {
		close(fd);
		IQRF_DEBUG_PRINTF("Unable to write '%s' into \"%s\". Reason: %s", buffer, path, strerror(errno));
		return IQRF_GPIO_ERROR_WRITE_FAILED;
	}
	close(fd);
	return IQRF_GPIO_ERROR_OK;
#endif
}

iqrf_gpio_error_t iqrf_gpio_unexport(int64_t pin) {
#ifdef WIN32
	return IQRF_GPIO_ERROR_OK;
#else
	if (pin < 0) {
		IQRF_DEBUG_PRINTF("Invalid GPIO pin number: %"PRId64, pin);
		return IQRF_GPIO_ERROR_INVALID_PIN;
	}
	char *path = IQRF_GPIO_SYSFS_BASE_PATH"unexport";
	int fd = open(path, O_WRONLY);
	if (fd == -1) {
		IQRF_DEBUG_PRINTF("Unable to open path \"%s\". Reason: %s", path, strerror(errno));
		return IQRF_GPIO_ERROR_OPEN_FAILED;
	}
	char buffer[IQRF_GPIO_PIN_BUFFER_SIZE] = "";
	snprintf(buffer, IQRF_GPIO_PIN_BUFFER_SIZE, "%"PRId64, pin);
	ssize_t writtenSize = write(fd, buffer, IQRF_GPIO_PIN_BUFFER_SIZE);
	if (writtenSize == -1) {
		close(fd);
		IQRF_DEBUG_PRINTF("Unable to write '%s' into \"%s\". Reason: %s", buffer, path, strerror(errno));
		return IQRF_GPIO_ERROR_WRITE_FAILED;
	}
	close(fd);
	return IQRF_GPIO_ERROR_OK;
#endif
}

iqrf_gpio_error_t iqrf_gpio_get_direction(int64_t pin, iqrf_gpio_direction_t *direction) {
#ifdef WIN32
	return IQRF_GPIO_ERROR_OK;
#else
	if (direction == NULL) {
		return IQRF_GPIO_ERROR_NULL_POINTER;
	}
	if (pin < 0) {
		IQRF_DEBUG_PRINTF("Invalid GPIO pin number: %"PRId64, pin);
		return IQRF_GPIO_ERROR_INVALID_PIN;
	}
	char path[IQRF_GPIO_SYSFS_BUFFER_SIZE] = "";
	iqrf_gpio_create_sysfs_path(pin, IQRF_GPIO_ACTION_DIRECTION, path);
	int fd = open(path, O_RDONLY);
	if (fd == -1) {
		IQRF_DEBUG_PRINTF("Unable to open path \"%s\". Reason: %s", path, strerror(errno));
		return IQRF_GPIO_ERROR_OPEN_FAILED;
	}
	char buffer[IQRF_GPIO_DIRECTION_BUFFER_SIZE] = "";
	ssize_t readSize = read(fd, buffer, IQRF_GPIO_DIRECTION_BUFFER_SIZE);
	if (readSize == -1) {
		close(fd);
		IQRF_DEBUG_PRINTF("Unable to read from %s", path);
		return IQRF_GPIO_ERROR_WRITE_FAILED;
	}
	if (strncmp(buffer, IQRF_GPIO_DIRECTION_IN_STR, IQRF_GPIO_DIRECTION_BUFFER_SIZE - 1) == 0) {
		*direction = IQRF_GPIO_DIRECTION_IN;
	} else if (strncmp(buffer, IQRF_GPIO_DIRECTION_OUT_STR, IQRF_GPIO_DIRECTION_BUFFER_SIZE - 1) == 0) {
		*direction = IQRF_GPIO_DIRECTION_OUT;
	} else {
		*direction = IQRF_GPIO_DIRECTION_UNKNOWN;
	}
	close(fd);
	return IQRF_GPIO_ERROR_OK;
#endif
}

iqrf_gpio_error_t iqrf_gpio_set_direction(int64_t pin, iqrf_gpio_direction_t direction) {
#ifdef WIN32
	return IQRF_GPIO_ERROR_OK;
#else
	if (pin < 0) {
		IQRF_DEBUG_PRINTF("Invalid GPIO pin number: %"PRId64, pin);
		return IQRF_GPIO_ERROR_INVALID_PIN;
	}
	char path[IQRF_GPIO_SYSFS_BUFFER_SIZE] = "";
	iqrf_gpio_create_sysfs_path(pin, IQRF_GPIO_ACTION_DIRECTION, path);
	int fd = open(path, O_WRONLY);
	if (fd == -1) {
		IQRF_DEBUG_PRINTF("Unable to open path \"%s\". Reason: %s", path, strerror(errno));
		return IQRF_GPIO_ERROR_OPEN_FAILED;
	}
	const char *buffer = direction == IQRF_GPIO_DIRECTION_IN ? IQRF_GPIO_DIRECTION_IN_STR : IQRF_GPIO_DIRECTION_OUT_STR;
	ssize_t writtenSize = write(fd, buffer, strlen(buffer));
	if (writtenSize == -1) {
		close(fd);
		IQRF_DEBUG_PRINTF("Unable to write '%s' into \"%s\". Reason: %s", buffer, path, strerror(errno));
		return IQRF_GPIO_ERROR_WRITE_FAILED;
	}
	close(fd);
	return IQRF_GPIO_ERROR_OK;
#endif
}

iqrf_gpio_error_t iqrf_gpio_get_value(int64_t pin, bool *value) {
#ifdef WIN32
	return IQRF_GPIO_ERROR_OK;
#else
	if (value == NULL) {
		return IQRF_GPIO_ERROR_NULL_POINTER;
	}
	if (pin < 0) {
		IQRF_DEBUG_PRINTF("Invalid GPIO pin number: %"PRId64, pin);
		return IQRF_GPIO_ERROR_INVALID_PIN;
	}
	char path[IQRF_GPIO_SYSFS_BUFFER_SIZE] = "";
	iqrf_gpio_create_sysfs_path(pin, IQRF_GPIO_ACTION_VALUE, path);
	int fd = open(path, O_RDONLY);
	if (fd == -1) {
		IQRF_DEBUG_PRINTF("Unable to open path \"%s\". Reason: %s", path, strerror(errno));
		return IQRF_GPIO_ERROR_OPEN_FAILED;
	}
	char buffer[IQRF_GPIO_VALUE_BUFFER_SIZE] = "";
	ssize_t readSize = read(fd, buffer, IQRF_GPIO_VALUE_BUFFER_SIZE);
	if (readSize == -1) {
		close(fd);
		IQRF_DEBUG_PRINTF("Unable to read from %s", path);
		return IQRF_GPIO_ERROR_WRITE_FAILED;
	}
	*value = strncmp(buffer, "1", IQRF_GPIO_VALUE_BUFFER_SIZE - 1) == 0;
	close(fd);
	return IQRF_GPIO_ERROR_OK;
#endif
}

iqrf_gpio_error_t iqrf_gpio_set_value(int64_t pin, bool value) {
#ifdef WIN32
	return IQRF_GPIO_ERROR_OK;
#else
	if (pin < 0) {
		IQRF_DEBUG_PRINTF("Invalid GPIO pin number: %"PRId64, pin);
		return IQRF_GPIO_ERROR_INVALID_PIN;
	}
	char path[IQRF_GPIO_SYSFS_BUFFER_SIZE] = "";
	iqrf_gpio_create_sysfs_path(pin, IQRF_GPIO_ACTION_VALUE, path);
	int fd = open(path, O_WRONLY);
	if (fd == -1) {
		IQRF_DEBUG_PRINTF("Unable to open path \"%s\". Reason: %s", path, strerror(errno));
		return IQRF_GPIO_ERROR_OPEN_FAILED;
	}
	const char *buffer = value ? "1" : "0";
	ssize_t writtenSize = write(fd, buffer, 2);
	if (writtenSize == -1) {
		close(fd);
		IQRF_DEBUG_PRINTF("Unable to write '%s' into \"%s\". Reason: %s", buffer, path, strerror(errno));
		return IQRF_GPIO_ERROR_WRITE_FAILED;
	}
	close(fd);
	return IQRF_GPIO_ERROR_OK;
#endif
}

iqrf_gpio_error_t iqrf_gpio_init(int64_t pin, iqrf_gpio_direction_t direction, bool initialValue) {
#ifdef WIN32
	return IQRF_GPIO_ERROR_OK;
#else
	iqrf_gpio_error_t error = iqrf_gpio_export(pin);
	if (error) {
		return error;
	}

	char *directionStr = direction == IQRF_GPIO_DIRECTION_IN ? "input" : "output";
	for (uint8_t i = 1; i <= 10; i++) {
		error = iqrf_gpio_set_direction(pin, direction);
		if (!error) {
			IQRF_DEBUG_PRINTF("GPIO pin #%"PRId64" direction \"%s\" is successfully set. Attempt: %d", pin, directionStr, i);
			break;
		}
		IQRF_DEBUG_PRINTF("Failed to set direction \"%s\" on GPIO pin #%"PRId64". Wait for 100 ms to next try: %d", directionStr, pin, i);
		SLEEP(100);
	}

	if (direction == IQRF_GPIO_DIRECTION_OUT) {
		error = iqrf_gpio_set_value(pin, initialValue);
		if (error) {
			return error;
		}
	}

	return IQRF_GPIO_ERROR_OK;
#endif
}

iqrf_gpio_error_t iqrf_gpio_init_input(int64_t pin) {
	return iqrf_gpio_init(pin, IQRF_GPIO_DIRECTION_IN, false);
}

iqrf_gpio_error_t iqrf_gpio_init_output(int64_t pin, bool initialValue) {
	return iqrf_gpio_init(pin, IQRF_GPIO_DIRECTION_OUT, initialValue);
}
