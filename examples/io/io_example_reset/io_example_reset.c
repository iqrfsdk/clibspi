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
const long INTERVAL_MS = 10 * NANO_SECOND_MULTIPLIER;

int main(void) {
    int ret = 0;

    //struct timespec sleepValue = {0, INTERVAL_MS};

    printf("IQRF module reset example \n");

    // switch off the power
    ret = gpio_setup(RESET_GPIO, GPIO_DIRECTION_OUT, 0);
    if (ret)
	    return;

    // sleep for 50 ms
    //nanosleep(&sleepValue, NULL);
    SLEEP(10);
        
    // switch on the power
    ret = gpio_setValue(RESET_GPIO, 1);

    // finish the library
    gpio_cleanup(RESET_GPIO);

    return 0;
}
