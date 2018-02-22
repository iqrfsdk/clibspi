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

#define SYSFS_GPIO_EXPORTS

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "sysfs_gpio.h"
#include <stdlib.h>

#include "sleepWrapper.h"

#ifdef WIN32
#define snprintf _snprintf
#endif

/* constants */
#define GPIO_BASE_PATH "/sys/class/gpio"
#define GPIO_EXPORT_PATH GPIO_BASE_PATH"/export"
#define GPIO_UNEXPORT_PATH GPIO_BASE_PATH"/unexport"

#define GPIO_DIRECTION_STR "direction"
#define GPIO_VALUE_STR "value"
#define GPIO_UNEXPORT_STR "unexport"

/* gpio gpio_getDirection state */
#define GPIO_DIRECTION_IN_STR "in"
#define GPIO_DIRECTION_OUT_STR "out"

/**
* Setup GPIO path
*
* @param [in]	gpio		GPIO
* @param [in]	action		action
* @param [in]	target		target
* @param [in]	len			length
*
*/
static void setup_gpio_path(const int gpio, const char *action, char *target, int len)
{
	snprintf(target, len, GPIO_BASE_PATH"/gpio%d/%s", gpio, action);
}

/**
* Writes data stored in @c fd file to buffer.
*
* @param [in]	fd		file to write.
* @param [in]	buf		buffer where file is written
*
* @return	-1 = error during write
* @return	0 = file was successfully written
*/
static int write_data(FILE *fd, const char *buf)
{
	int ret = 0;

	ret = fwrite(buf, 1, strlen(buf), fd);
	if (ret != strlen(buf)) {
		printf("Error during writing to file\n");
		ret = -1;
	}
	else {
		ret = 0;
	}

	return ret;
}

/**
* Exports the GPIO pin to the GPIO array
*
* @param num	GPIO number
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int gpio_export(int num)
{
	FILE *fd = fopen(GPIO_EXPORT_PATH, "w");
	char buf[5];
	int ret;

	if (!fd) {
		printf("Error during opening file: %s\n", strerror(errno));
		return -1;
	}

	snprintf(buf, sizeof(buf), "%d", num);
	ret = write_data(fd, buf);
	if (ret)
		goto err;

err:
	fclose(fd);
	return ret;
}

/**
* Unexports the GPIO pin from the GPIO array
*
* @param num	GPIO number
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int gpio_unexport(int num)
{
	FILE *fd = fopen(GPIO_UNEXPORT_PATH, "w");
	char buf[5];
	int ret;

	if (!fd) {
		printf("Error during opening file: %s\n", strerror(errno));
		return -1;
	}

	snprintf(buf, sizeof(buf), "%d", num);
	ret = write_data(fd, buf);
	if (ret)
		goto err;

err:
	fclose(fd);
	return ret;
}


/**
* Gets direction of the GPIO
*
* @param gpio	GPIO
* @param dir	GPIO direction  = GPIO_DIRECTION_IN or GPIO_DIRECTION_OUT
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int gpio_setDirection(int gpio, gpio_direction dir)
{
	char path[50];
	char buf[4];
	FILE *fd = NULL;
	int ret;

	setup_gpio_path(gpio, GPIO_DIRECTION_STR, path, sizeof(path));

	fd = fopen(path, "w");

	if (!fd) {
		printf("Error during opening file (set direction): %s  %s\n", path, strerror(errno));
		return -1;
	}
	if (dir == GPIO_DIRECTION_IN) {
		strncpy(buf, GPIO_DIRECTION_IN_STR, sizeof(buf));
	}
	else if (dir == GPIO_DIRECTION_OUT) {
		strncpy(buf, GPIO_DIRECTION_OUT_STR, sizeof(buf));
	}

	ret = write_data(fd, buf);
	if (ret)
		goto err;

err:
	fclose(fd);
	return ret;
}

/**
* Gets direction of the GPIO
*
* @param gpio	GPIO
*
* @return	GPIO_DIRECTION_IN = GPIO set to input.
* @return	GPIO_DIRECTION_OUT = GPIO set to output.
*/
gpio_direction gpio_getDirection(int gpio)
{
	char path[50];
	char buf[4];
	FILE *fd = NULL;
	int ret;
	gpio_direction dir;

	setup_gpio_path(gpio, GPIO_DIRECTION_STR, path, sizeof(path));

	fd = fopen(path, "r");

	if (!fd) {
		printf("Error during opening file (get direction): %s\n", strerror(errno));
		return -1;
	}

	ret = fread(buf, 1, sizeof(buf), fd);
	if (!ret) {
		printf("Error during reading file\n");
		ret = -1;
		goto err;
	}

	if (!strcmp(buf, GPIO_DIRECTION_IN_STR))
		dir = GPIO_DIRECTION_IN;
	else if (!strcmp(buf, GPIO_DIRECTION_OUT_STR))
		dir = GPIO_DIRECTION_OUT;

	ret = 0;

err:
	fclose(fd);
	return ret;

}

/**
* Sets GPIO value
*
* @param gpio	GPIO
* @param val	value
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int gpio_setValue(int gpio, int val)
{
	char path[50];
	char buf[2];
	FILE *fd = NULL;
	int ret;

	setup_gpio_path(gpio, GPIO_VALUE_STR, path, sizeof(path));

	fd = fopen(path, "w");

	if (!fd) {
		printf("Error during opening file: %s\n", strerror(errno));
		return -1;
	}

	snprintf(buf, sizeof(buf), "%d", val);
	ret = write_data(fd, buf);
	if (ret)
		goto err;

err:
	fclose(fd);
	return ret;
}

/**
* Gets GPIO value
*
* @param gpio	GPIO
*
* @return	GPIO value
*/
int gpio_getValue(int gpio)
{
	char path[50];
	char buf[2];
	FILE *fd = NULL;
	int ret;

	setup_gpio_path(gpio, GPIO_VALUE_STR, path, sizeof(path));

	fd = fopen(path, "r");

	if (!fd) {
		printf("Error during opening file: %s\n", strerror(errno));
		return -1;
	}

	ret = fread(buf, 1, sizeof(buf), fd);
	if (!ret) {
		printf("Error during reading file\n");
		ret = -1;
		goto err;
	}

	ret = strtol(buf, NULL, 10);

err:
	fclose(fd);
	return ret;
}

/**
* Setup GPIO
*
* @param gpio	GPIO
* @param dir	GPIO direction
* @param val	GPIO value
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int gpio_setup(int gpio, gpio_direction dir, int val)
{
	int ret;

	ret = gpio_export(gpio);
	if (ret)
		return ret;

	int i;
	for (i = 1; i <= 10; i++) {
	  ret = gpio_setDirection(gpio, dir);
	  if (!ret) {
		printf("gpio_setup() setDir success: %d\n", i);
		break;
	  }
	  else {
		printf("gpio_setup() setDir failed wait for 100 ms to next try: %d\n", i);
		SLEEP(100);
	  }
	}

	// set gpio_getValue when output gpio_getDirection
	if (dir == GPIO_DIRECTION_OUT) {
		ret = gpio_setValue(gpio, val);
		if (ret)
			return ret;
	}

	return ret;
}

/**
* Cleans and releases GPIO
*
* @param gpio	GPIO
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
//void gpio_cleanup(int gpio)
int gpio_cleanup(int gpio)
{
	//	gpio_setValue(gpio, 0);
	return (gpio_unexport(gpio));
}
