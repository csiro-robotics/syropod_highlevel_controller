// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot

//
// Project configuration header. This is a generated header; do not modify
// it directly. Instead, modify the config.h.in version and run CMake again.
//
#ifndef SHCCONFIG_H_
#define SHCCONFIG_H_

// MSVC does not define common maths constants like M_PI by default.
// Make sure they are defined if we include cmath
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif  // _USE_MATH_DEFINES
#ifndef NOMINMAX
// Disable MSVC min() and max() macros in favour of of function implementations.
#define NOMINMAX
#endif  // NOMINMAX
#ifndef NOMINMAX
#define NOMINMAX
#endif  // NOMINMAX

#ifdef _MSC_VER
// Avoid dubious security warnings for plenty of legitimate code
#ifndef _SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif  // _SCL_SECURE_NO_WARNINGS
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif  // _CRT_SECURE_NO_WARNINGS
//#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif  // _MSC_VER

#endif  // SHCCONFIG_H_
