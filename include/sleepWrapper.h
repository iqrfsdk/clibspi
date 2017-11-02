#pragma once

#ifndef WIN32
#include <unistd.h>
#define SLEEP(ms) usleep(1000*ms)
#else
#include "Windows.h"
#define SLEEP(ms) Sleep(ms)
#endif

