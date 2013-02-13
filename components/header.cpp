/*
 *
 * Header for high performance Arduino Digital I/O
 *
 * Automatically generated from the Arduino library setup (boards.txt & pins_arduino.h)
 *
 * See the accompanying file README.md for documentation.
 *
 * ****
 *
 * This header is a derived work of the Arduino microcontroller libraries, which are
 * licensed under LGPL. Although as a header file it is not bound by the same usage
 * clauses as the library itself (see "3. Object Code Incorporating Material from
 * Library Header Files.)"
 *
 * Note that although the code generated functions below here look horrific,
 * they're written to inline only very small subsets of themselves at compile
 * time (they generate single port-register instructions when the parameters
 * are constant.)
 *
 *
 */

#ifdef __AVR__
#ifndef _DIGITALIO_PERFORMANCE
#define _DIGITALIO_PERFORMANCE

#include "Arduino.h"
#include <util/atomic.h>

