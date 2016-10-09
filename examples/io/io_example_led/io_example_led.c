/*
 * Copyright 2015 MICRORISC s.r.o.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
//#include <linux/types.h>
#include <sysfs_gpio.h>
#include <machines_def.h>
#include "sleepWrapper.h"

#define NANO_SECOND_MULTIPLIER  1000000  // 1 millisecond = 1,000,000 nanoseconds
const long INTERVAL_MS = 500 * NANO_SECOND_MULTIPLIER;

int main(void) {
  int cycle = 0;
  int value = 0;
  int ret;

  //struct timespec sleepValue = {0, INTERVAL_MS};

  printf("LED usage example \n");

  // enable access to GPIOs
  ret = gpio_setup(LED_GPIO, GPIO_DIRECTION_OUT, value);
  if (ret) {
	  printf("LED_GPIO error \n");
	  return -1;
  }

  printf("LED_GPIO ready \n");

  do {
    value ^= 1;
    // toggle gpio_getValue
    gpio_setValue(LED_GPIO, value);
    //nanosleep(&sleepValue, NULL);
    SLEEP(100);
  } while (cycle++ <= 10);

  printf("LED_GPIO cleanup \n");
  gpio_cleanup(LED_GPIO);
  return 0;
}
