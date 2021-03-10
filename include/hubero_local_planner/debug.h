/*
 * debug.h
 *
 *  Created on: Mar 10, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_DEBUG_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_DEBUG_H_

// https://stackoverflow.com/questions/1644868/define-macro-for-debug-printing-in-c
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define _template_debug_print_basic_(enable, fmt, ...) do { if (enable) printf("%s(): " fmt, __func__, ##__VA_ARGS__); } while (0)
#define _template_debug_print_warn_(enable, fmt, ...) do { if (enable) printf("%s(): " ANSI_COLOR_YELLOW fmt ANSI_COLOR_RESET, __func__, ##__VA_ARGS__); } while (0)
#define _template_debug_print_err_(enable, fmt, ...) do { if (enable) printf("%s(): " ANSI_COLOR_RED fmt ANSI_COLOR_RESET, __func__, ##__VA_ARGS__); } while (0)

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_DEBUG_H_ */
