#pragma once

#ifdef WIN32
#   define LIB_EXPORT_SHARED __declspec(dllexport)
#   define LIB_IMPORT_SHARED __declspec(dllimport)
#else // POSIX
#   define LIB_EXPORT_SHARED __attribute__ ((visibility ("default")))
#   define LIB_IMPORT_SHARED __attribute__ ((visibility ("default")))
#endif
