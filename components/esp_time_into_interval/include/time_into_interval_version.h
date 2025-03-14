/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file time_into_interval_version.h
 * @defgroup schedule
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TIME_INTO_INTERVAL_VERSION_H__
#define __TIME_INTO_INTERVAL_VERSION_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

/** Major version number (X.x.x) */
#define TIME_INTO_INTERVAL_FW_VERSION_MAJOR 1
/** Minor version number (x.X.x) */
#define TIME_INTO_INTERVAL_FW_VERSION_MINOR 1
/** Patch version number (x.x.X) */
#define TIME_INTO_INTERVAL_FW_VERSION_PATCH 7


/**
 * public macro definitions
 */

/** 
 * Macro to print x parameter as a string i.e. enclose x in double quotes. 
 */
#define STR_QUOTES( x ) #x

/** 
 * Macro to create a string of x parameter with all macros fully expanded. 
 */                 
#define STR( x ) STR_QUOTES( x )

/** 
 * Macro to generate current firmware version numbers (major, minor, patch) into a string that is formatted as X.X.X (e.g. 4.0.0). 
 */
#define TIME_INTO_INTERVAL_FW_VERSION_STR                        \
                        STR( TIME_INTO_INTERVAL_FW_VERSION_MAJOR ) "." \
                        STR( TIME_INTO_INTERVAL_FW_VERSION_MINOR ) "." \
                        STR( TIME_INTO_INTERVAL_FW_VERSION_PATCH )

/** 
 * Macro to convert firmware version parameters (major, minor, patch numbers) into an integer (`int32_t`) 
 * value that can be used for comparison purposes.
 * 
 * As an example, [COMPONENT]_FW_VERSION_INT32 >= [COMPONENT]_FW_VERSION_PARAMS_INT32(4, 0, 0).
 */
#define TIME_INTO_INTERVAL_FW_VERSION_PARAMS_INT32( major, minor, patch )        \
        ((major << 16) | (minor << 8) | (patch))

/**
 * Macro to generate current firmware version numbers (major, minor, patch) as an integer (`int32_t`) value that can 
 * be used for comparison purposes.
 * 
 * As an example, [COMPONENT]_FW_VERSION_INT32 >= [COMPONENT]_FW_VERSION_PARAMS_INT32(4, 0, 0).
 */
#define TIME_INTO_INTERVAL_FW_VERSION_INT32                                      \
                TIME_INTO_INTERVAL_FW_VERSION_PARAMS_INT32(TIME_INTO_INTERVAL_FW_VERSION_MAJOR,   \
                                        TIME_INTO_INTERVAL_FW_VERSION_MINOR,   \
                                        TIME_INTO_INTERVAL_FW_VERSION_PATCH)





#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __TIME_INTO_INTERVAL_VERSION_H__
