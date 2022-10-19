#pragma once

#ifdef NDEBUG
#ifndef IQRF_DEBUG_PRINT
#define IQRF_DEBUG_PRINT(fmt, ...)
#endif
#ifndef IQRF_DEBUG_PRINTF
#define IQRF_DEBUG_PRINTF(fmt, ...)
#endif
#else
#ifndef IQRF_DEBUG_PRINT
#define IQRF_DEBUG_PRINT(str) do { \
    fprintf(stderr, "%s:%d - %s(): " str "\n", __FILE__, __LINE__, __func__); \
} while (0)
#endif
#ifndef IQRF_DEBUG_PRINTF
#define IQRF_DEBUG_PRINTF(fmt, ...) do { \
    fprintf(stderr, "%s:%d - %s(): " fmt "\n", __FILE__, __LINE__, __func__, __VA_ARGS__); \
} while (0)
#endif
#endif
