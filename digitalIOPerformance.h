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

/* Arduino board:
 *   mini | nano | bt
 *   Arduino Mini w/ ATmega168 | Arduino Nano w/ ATmega168 | Arduino BT w/ ATmega168
 *   MCU: atmega168
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 8 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x94 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR2A &= ~COM2B1;
  else if(pin == 5) TCCR0A &= ~COM0B1;
  else if(pin == 6) TCCR0A &= ~COM0A1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2A &= ~COM2A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   pro | lilypad
 *   Arduino Pro or Pro Mini (3.3V, 8 MHz) w/ ATmega168 | LilyPad Arduino w/ ATmega168
 *   MCU: atmega168
 */
#if defined(F_CPU) && (F_CPU+0) == 8000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 6 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x94 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR2A &= ~COM2B1;
  else if(pin == 5) TCCR0A &= ~COM0B1;
  else if(pin == 6) TCCR0A &= ~COM0A1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2A &= ~COM2A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   lilypad328 | pro328
 *   LilyPad Arduino w/ ATmega328 | Arduino Pro or Pro Mini (3.3V, 8 MHz) w/ ATmega328
 *   MCU: atmega328p
 */
#if defined(F_CPU) && (F_CPU+0) == 8000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 6 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR2A &= ~COM2B1;
  else if(pin == 5) TCCR0A &= ~COM0B1;
  else if(pin == 6) TCCR0A &= ~COM0A1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2A &= ~COM2A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   atmega8
 *   Arduino NG or older w/ ATmega8
 *   MCU: atmega8
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 6 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x93 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2 &= ~COM21;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   pro5v | atmega168 | diecimila
 *   Arduino Pro or Pro Mini (5V, 16 MHz) w/ ATmega168 | Arduino NG or older w/ ATmega168 | Arduino Diecimila or Duemilanove w/ ATmega168
 *   MCU: atmega168
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 6 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x94 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR2A &= ~COM2B1;
  else if(pin == 5) TCCR0A &= ~COM0B1;
  else if(pin == 6) TCCR0A &= ~COM0A1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2A &= ~COM2A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   fio
 *   Arduino Fio
 *   MCU: atmega328p
 */
#if defined(F_CPU) && (F_CPU+0) == 8000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 8 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR2A &= ~COM2B1;
  else if(pin == 5) TCCR0A &= ~COM0B1;
  else if(pin == 6) TCCR0A &= ~COM0A1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2A &= ~COM2A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   mega2560
 *   Arduino Mega 2560 or Mega ADK
 *   MCU: atmega2560
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 16 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x98 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRE |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (0));
    PORTE &= ~(1 << (0));
  } else if(pin == 0) DDRE &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRE |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (1));
    PORTE &= ~(1 << (1));
  } else if(pin == 1) DDRE &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRE |= (1 << (4));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (4));
    PORTE &= ~(1 << (4));
  } else if(pin == 2) DDRE &= ~(1 << (4));
  else if(pin == 3 && mode == INPUT) DDRE |= (1 << (5));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (5));
    PORTE &= ~(1 << (5));
  } else if(pin == 3) DDRE &= ~(1 << (5));
  else if(pin == 4 && mode == INPUT) DDRG |= (1 << (5));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (5));
    PORTG &= ~(1 << (5));
  } else if(pin == 4) DDRG &= ~(1 << (5));
  else if(pin == 5 && mode == INPUT) DDRE |= (1 << (3));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (3));
    PORTE &= ~(1 << (3));
  } else if(pin == 5) DDRE &= ~(1 << (3));
  else if(pin == 6 && mode == INPUT) DDRH |= (1 << (3));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (3));
    PORTH &= ~(1 << (3));
  } else if(pin == 6) DDRH &= ~(1 << (3));
  else if(pin == 7 && mode == INPUT) DDRH |= (1 << (4));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (4));
    PORTH &= ~(1 << (4));
  } else if(pin == 7) DDRH &= ~(1 << (4));
  else if(pin == 8 && mode == INPUT) DDRH |= (1 << (5));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (5));
    PORTH &= ~(1 << (5));
  } else if(pin == 8) DDRH &= ~(1 << (5));
  else if(pin == 9 && mode == INPUT) DDRH |= (1 << (6));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (6));
    PORTH &= ~(1 << (6));
  } else if(pin == 9) DDRH &= ~(1 << (6));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 10) DDRB &= ~(1 << (4));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 11) DDRB &= ~(1 << (5));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 12) DDRB &= ~(1 << (6));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (7));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (7));
    PORTB &= ~(1 << (7));
  } else if(pin == 13) DDRB &= ~(1 << (7));
  else if(pin == 14 && mode == INPUT) DDRJ |= (1 << (1));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRJ |= (1 << (1));
    PORTJ &= ~(1 << (1));
  } else if(pin == 14) DDRJ &= ~(1 << (1));
  else if(pin == 15 && mode == INPUT) DDRJ |= (1 << (0));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRJ |= (1 << (0));
    PORTJ &= ~(1 << (0));
  } else if(pin == 15) DDRJ &= ~(1 << (0));
  else if(pin == 16 && mode == INPUT) DDRH |= (1 << (1));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (1));
    PORTH &= ~(1 << (1));
  } else if(pin == 16) DDRH &= ~(1 << (1));
  else if(pin == 17 && mode == INPUT) DDRH |= (1 << (0));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (0));
    PORTH &= ~(1 << (0));
  } else if(pin == 17) DDRH &= ~(1 << (0));
  else if(pin == 18 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 18) DDRD &= ~(1 << (3));
  else if(pin == 19 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 19) DDRD &= ~(1 << (2));
  else if(pin == 20 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 20 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 20) DDRD &= ~(1 << (1));
  else if(pin == 21 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 21 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 21) DDRD &= ~(1 << (0));
  else if(pin == 22 && mode == INPUT) DDRA |= (1 << (0));
  else if(pin == 22 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (0));
    PORTA &= ~(1 << (0));
  } else if(pin == 22) DDRA &= ~(1 << (0));
  else if(pin == 23 && mode == INPUT) DDRA |= (1 << (1));
  else if(pin == 23 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (1));
    PORTA &= ~(1 << (1));
  } else if(pin == 23) DDRA &= ~(1 << (1));
  else if(pin == 24 && mode == INPUT) DDRA |= (1 << (2));
  else if(pin == 24 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (2));
    PORTA &= ~(1 << (2));
  } else if(pin == 24) DDRA &= ~(1 << (2));
  else if(pin == 25 && mode == INPUT) DDRA |= (1 << (3));
  else if(pin == 25 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (3));
    PORTA &= ~(1 << (3));
  } else if(pin == 25) DDRA &= ~(1 << (3));
  else if(pin == 26 && mode == INPUT) DDRA |= (1 << (4));
  else if(pin == 26 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (4));
    PORTA &= ~(1 << (4));
  } else if(pin == 26) DDRA &= ~(1 << (4));
  else if(pin == 27 && mode == INPUT) DDRA |= (1 << (5));
  else if(pin == 27 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (5));
    PORTA &= ~(1 << (5));
  } else if(pin == 27) DDRA &= ~(1 << (5));
  else if(pin == 28 && mode == INPUT) DDRA |= (1 << (6));
  else if(pin == 28 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (6));
    PORTA &= ~(1 << (6));
  } else if(pin == 28) DDRA &= ~(1 << (6));
  else if(pin == 29 && mode == INPUT) DDRA |= (1 << (7));
  else if(pin == 29 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (7));
    PORTA &= ~(1 << (7));
  } else if(pin == 29) DDRA &= ~(1 << (7));
  else if(pin == 30 && mode == INPUT) DDRC |= (1 << (7));
  else if(pin == 30 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (7));
    PORTC &= ~(1 << (7));
  } else if(pin == 30) DDRC &= ~(1 << (7));
  else if(pin == 31 && mode == INPUT) DDRC |= (1 << (6));
  else if(pin == 31 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (6));
    PORTC &= ~(1 << (6));
  } else if(pin == 31) DDRC &= ~(1 << (6));
  else if(pin == 32 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 32 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 32) DDRC &= ~(1 << (5));
  else if(pin == 33 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 33 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 33) DDRC &= ~(1 << (4));
  else if(pin == 34 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 34 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 34) DDRC &= ~(1 << (3));
  else if(pin == 35 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 35 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 35) DDRC &= ~(1 << (2));
  else if(pin == 36 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 36 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 36) DDRC &= ~(1 << (1));
  else if(pin == 37 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 37 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 37) DDRC &= ~(1 << (0));
  else if(pin == 38 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 38 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 38) DDRD &= ~(1 << (7));
  else if(pin == 39 && mode == INPUT) DDRG |= (1 << (2));
  else if(pin == 39 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (2));
    PORTG &= ~(1 << (2));
  } else if(pin == 39) DDRG &= ~(1 << (2));
  else if(pin == 40 && mode == INPUT) DDRG |= (1 << (1));
  else if(pin == 40 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (1));
    PORTG &= ~(1 << (1));
  } else if(pin == 40) DDRG &= ~(1 << (1));
  else if(pin == 41 && mode == INPUT) DDRG |= (1 << (0));
  else if(pin == 41 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (0));
    PORTG &= ~(1 << (0));
  } else if(pin == 41) DDRG &= ~(1 << (0));
  else if(pin == 42 && mode == INPUT) DDRL |= (1 << (7));
  else if(pin == 42 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (7));
    PORTL &= ~(1 << (7));
  } else if(pin == 42) DDRL &= ~(1 << (7));
  else if(pin == 43 && mode == INPUT) DDRL |= (1 << (6));
  else if(pin == 43 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (6));
    PORTL &= ~(1 << (6));
  } else if(pin == 43) DDRL &= ~(1 << (6));
  else if(pin == 44 && mode == INPUT) DDRL |= (1 << (5));
  else if(pin == 44 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (5));
    PORTL &= ~(1 << (5));
  } else if(pin == 44) DDRL &= ~(1 << (5));
  else if(pin == 45 && mode == INPUT) DDRL |= (1 << (4));
  else if(pin == 45 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (4));
    PORTL &= ~(1 << (4));
  } else if(pin == 45) DDRL &= ~(1 << (4));
  else if(pin == 46 && mode == INPUT) DDRL |= (1 << (3));
  else if(pin == 46 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (3));
    PORTL &= ~(1 << (3));
  } else if(pin == 46) DDRL &= ~(1 << (3));
  else if(pin == 47 && mode == INPUT) DDRL |= (1 << (2));
  else if(pin == 47 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (2));
    PORTL &= ~(1 << (2));
  } else if(pin == 47) DDRL &= ~(1 << (2));
  else if(pin == 48 && mode == INPUT) DDRL |= (1 << (1));
  else if(pin == 48 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (1));
    PORTL &= ~(1 << (1));
  } else if(pin == 48) DDRL &= ~(1 << (1));
  else if(pin == 49 && mode == INPUT) DDRL |= (1 << (0));
  else if(pin == 49 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (0));
    PORTL &= ~(1 << (0));
  } else if(pin == 49) DDRL &= ~(1 << (0));
  else if(pin == 50 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 50 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 50) DDRB &= ~(1 << (3));
  else if(pin == 51 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 51 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 51) DDRB &= ~(1 << (2));
  else if(pin == 52 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 52 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 52) DDRB &= ~(1 << (1));
  else if(pin == 53 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 53 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 53) DDRB &= ~(1 << (0));
  else if(pin == 54 && mode == INPUT) DDRF |= (1 << (0));
  else if(pin == 54 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (0));
    PORTF &= ~(1 << (0));
  } else if(pin == 54) DDRF &= ~(1 << (0));
  else if(pin == 55 && mode == INPUT) DDRF |= (1 << (1));
  else if(pin == 55 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (1));
    PORTF &= ~(1 << (1));
  } else if(pin == 55) DDRF &= ~(1 << (1));
  else if(pin == 56 && mode == INPUT) DDRF |= (1 << (2));
  else if(pin == 56 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (2));
    PORTF &= ~(1 << (2));
  } else if(pin == 56) DDRF &= ~(1 << (2));
  else if(pin == 57 && mode == INPUT) DDRF |= (1 << (3));
  else if(pin == 57 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (3));
    PORTF &= ~(1 << (3));
  } else if(pin == 57) DDRF &= ~(1 << (3));
  else if(pin == 58 && mode == INPUT) DDRF |= (1 << (4));
  else if(pin == 58 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (4));
    PORTF &= ~(1 << (4));
  } else if(pin == 58) DDRF &= ~(1 << (4));
  else if(pin == 59 && mode == INPUT) DDRF |= (1 << (5));
  else if(pin == 59 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (5));
    PORTF &= ~(1 << (5));
  } else if(pin == 59) DDRF &= ~(1 << (5));
  else if(pin == 60 && mode == INPUT) DDRF |= (1 << (6));
  else if(pin == 60 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (6));
    PORTF &= ~(1 << (6));
  } else if(pin == 60) DDRF &= ~(1 << (6));
  else if(pin == 61 && mode == INPUT) DDRF |= (1 << (7));
  else if(pin == 61 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (7));
    PORTF &= ~(1 << (7));
  } else if(pin == 61) DDRF &= ~(1 << (7));
  else if(pin == 62 && mode == INPUT) DDRK |= (1 << (0));
  else if(pin == 62 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (0));
    PORTK &= ~(1 << (0));
  } else if(pin == 62) DDRK &= ~(1 << (0));
  else if(pin == 63 && mode == INPUT) DDRK |= (1 << (1));
  else if(pin == 63 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (1));
    PORTK &= ~(1 << (1));
  } else if(pin == 63) DDRK &= ~(1 << (1));
  else if(pin == 64 && mode == INPUT) DDRK |= (1 << (2));
  else if(pin == 64 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (2));
    PORTK &= ~(1 << (2));
  } else if(pin == 64) DDRK &= ~(1 << (2));
  else if(pin == 65 && mode == INPUT) DDRK |= (1 << (3));
  else if(pin == 65 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (3));
    PORTK &= ~(1 << (3));
  } else if(pin == 65) DDRK &= ~(1 << (3));
  else if(pin == 66 && mode == INPUT) DDRK |= (1 << (4));
  else if(pin == 66 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (4));
    PORTK &= ~(1 << (4));
  } else if(pin == 66) DDRK &= ~(1 << (4));
  else if(pin == 67 && mode == INPUT) DDRK |= (1 << (5));
  else if(pin == 67 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (5));
    PORTK &= ~(1 << (5));
  } else if(pin == 67) DDRK &= ~(1 << (5));
  else if(pin == 68 && mode == INPUT) DDRK |= (1 << (6));
  else if(pin == 68 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (6));
    PORTK &= ~(1 << (6));
  } else if(pin == 68) DDRK &= ~(1 << (6));
  else if(pin == 69 && mode == INPUT) DDRK |= (1 << (7));
  else if(pin == 69 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (7));
    PORTK &= ~(1 << (7));
  } else if(pin == 69) DDRK &= ~(1 << (7));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTE  |= (1 << (0));
  else if(pin == 0 && !value) PORTE &= ~(1 << (0));
  else if(pin == 1 && value) PORTE  |= (1 << (1));
  else if(pin == 1 && !value) PORTE &= ~(1 << (1));
  else if(pin == 2 && value) PORTE  |= (1 << (4));
  else if(pin == 2 && !value) PORTE &= ~(1 << (4));
  else if(pin == 3 && value) PORTE  |= (1 << (5));
  else if(pin == 3 && !value) PORTE &= ~(1 << (5));
  else if(pin == 4 && value) PORTG  |= (1 << (5));
  else if(pin == 4 && !value) PORTG &= ~(1 << (5));
  else if(pin == 5 && value) PORTE  |= (1 << (3));
  else if(pin == 5 && !value) PORTE &= ~(1 << (3));
  else if(pin == 6 && value) PORTH  |= (1 << (3));
  else if(pin == 6 && !value) PORTH &= ~(1 << (3));
  else if(pin == 7 && value) PORTH  |= (1 << (4));
  else if(pin == 7 && !value) PORTH &= ~(1 << (4));
  else if(pin == 8 && value) PORTH  |= (1 << (5));
  else if(pin == 8 && !value) PORTH &= ~(1 << (5));
  else if(pin == 9 && value) PORTH  |= (1 << (6));
  else if(pin == 9 && !value) PORTH &= ~(1 << (6));
  else if(pin == 10 && value) PORTB  |= (1 << (4));
  else if(pin == 10 && !value) PORTB &= ~(1 << (4));
  else if(pin == 11 && value) PORTB  |= (1 << (5));
  else if(pin == 11 && !value) PORTB &= ~(1 << (5));
  else if(pin == 12 && value) PORTB  |= (1 << (6));
  else if(pin == 12 && !value) PORTB &= ~(1 << (6));
  else if(pin == 13 && value) PORTB  |= (1 << (7));
  else if(pin == 13 && !value) PORTB &= ~(1 << (7));
  else if(pin == 14 && value) PORTJ  |= (1 << (1));
  else if(pin == 14 && !value) PORTJ &= ~(1 << (1));
  else if(pin == 15 && value) PORTJ  |= (1 << (0));
  else if(pin == 15 && !value) PORTJ &= ~(1 << (0));
  else if(pin == 16 && value) PORTH  |= (1 << (1));
  else if(pin == 16 && !value) PORTH &= ~(1 << (1));
  else if(pin == 17 && value) PORTH  |= (1 << (0));
  else if(pin == 17 && !value) PORTH &= ~(1 << (0));
  else if(pin == 18 && value) PORTD  |= (1 << (3));
  else if(pin == 18 && !value) PORTD &= ~(1 << (3));
  else if(pin == 19 && value) PORTD  |= (1 << (2));
  else if(pin == 19 && !value) PORTD &= ~(1 << (2));
  else if(pin == 20 && value) PORTD  |= (1 << (1));
  else if(pin == 20 && !value) PORTD &= ~(1 << (1));
  else if(pin == 21 && value) PORTD  |= (1 << (0));
  else if(pin == 21 && !value) PORTD &= ~(1 << (0));
  else if(pin == 22 && value) PORTA  |= (1 << (0));
  else if(pin == 22 && !value) PORTA &= ~(1 << (0));
  else if(pin == 23 && value) PORTA  |= (1 << (1));
  else if(pin == 23 && !value) PORTA &= ~(1 << (1));
  else if(pin == 24 && value) PORTA  |= (1 << (2));
  else if(pin == 24 && !value) PORTA &= ~(1 << (2));
  else if(pin == 25 && value) PORTA  |= (1 << (3));
  else if(pin == 25 && !value) PORTA &= ~(1 << (3));
  else if(pin == 26 && value) PORTA  |= (1 << (4));
  else if(pin == 26 && !value) PORTA &= ~(1 << (4));
  else if(pin == 27 && value) PORTA  |= (1 << (5));
  else if(pin == 27 && !value) PORTA &= ~(1 << (5));
  else if(pin == 28 && value) PORTA  |= (1 << (6));
  else if(pin == 28 && !value) PORTA &= ~(1 << (6));
  else if(pin == 29 && value) PORTA  |= (1 << (7));
  else if(pin == 29 && !value) PORTA &= ~(1 << (7));
  else if(pin == 30 && value) PORTC  |= (1 << (7));
  else if(pin == 30 && !value) PORTC &= ~(1 << (7));
  else if(pin == 31 && value) PORTC  |= (1 << (6));
  else if(pin == 31 && !value) PORTC &= ~(1 << (6));
  else if(pin == 32 && value) PORTC  |= (1 << (5));
  else if(pin == 32 && !value) PORTC &= ~(1 << (5));
  else if(pin == 33 && value) PORTC  |= (1 << (4));
  else if(pin == 33 && !value) PORTC &= ~(1 << (4));
  else if(pin == 34 && value) PORTC  |= (1 << (3));
  else if(pin == 34 && !value) PORTC &= ~(1 << (3));
  else if(pin == 35 && value) PORTC  |= (1 << (2));
  else if(pin == 35 && !value) PORTC &= ~(1 << (2));
  else if(pin == 36 && value) PORTC  |= (1 << (1));
  else if(pin == 36 && !value) PORTC &= ~(1 << (1));
  else if(pin == 37 && value) PORTC  |= (1 << (0));
  else if(pin == 37 && !value) PORTC &= ~(1 << (0));
  else if(pin == 38 && value) PORTD  |= (1 << (7));
  else if(pin == 38 && !value) PORTD &= ~(1 << (7));
  else if(pin == 39 && value) PORTG  |= (1 << (2));
  else if(pin == 39 && !value) PORTG &= ~(1 << (2));
  else if(pin == 40 && value) PORTG  |= (1 << (1));
  else if(pin == 40 && !value) PORTG &= ~(1 << (1));
  else if(pin == 41 && value) PORTG  |= (1 << (0));
  else if(pin == 41 && !value) PORTG &= ~(1 << (0));
  else if(pin == 42 && value) PORTL  |= (1 << (7));
  else if(pin == 42 && !value) PORTL &= ~(1 << (7));
  else if(pin == 43 && value) PORTL  |= (1 << (6));
  else if(pin == 43 && !value) PORTL &= ~(1 << (6));
  else if(pin == 44 && value) PORTL  |= (1 << (5));
  else if(pin == 44 && !value) PORTL &= ~(1 << (5));
  else if(pin == 45 && value) PORTL  |= (1 << (4));
  else if(pin == 45 && !value) PORTL &= ~(1 << (4));
  else if(pin == 46 && value) PORTL  |= (1 << (3));
  else if(pin == 46 && !value) PORTL &= ~(1 << (3));
  else if(pin == 47 && value) PORTL  |= (1 << (2));
  else if(pin == 47 && !value) PORTL &= ~(1 << (2));
  else if(pin == 48 && value) PORTL  |= (1 << (1));
  else if(pin == 48 && !value) PORTL &= ~(1 << (1));
  else if(pin == 49 && value) PORTL  |= (1 << (0));
  else if(pin == 49 && !value) PORTL &= ~(1 << (0));
  else if(pin == 50 && value) PORTB  |= (1 << (3));
  else if(pin == 50 && !value) PORTB &= ~(1 << (3));
  else if(pin == 51 && value) PORTB  |= (1 << (2));
  else if(pin == 51 && !value) PORTB &= ~(1 << (2));
  else if(pin == 52 && value) PORTB  |= (1 << (1));
  else if(pin == 52 && !value) PORTB &= ~(1 << (1));
  else if(pin == 53 && value) PORTB  |= (1 << (0));
  else if(pin == 53 && !value) PORTB &= ~(1 << (0));
  else if(pin == 54 && value) PORTF  |= (1 << (0));
  else if(pin == 54 && !value) PORTF &= ~(1 << (0));
  else if(pin == 55 && value) PORTF  |= (1 << (1));
  else if(pin == 55 && !value) PORTF &= ~(1 << (1));
  else if(pin == 56 && value) PORTF  |= (1 << (2));
  else if(pin == 56 && !value) PORTF &= ~(1 << (2));
  else if(pin == 57 && value) PORTF  |= (1 << (3));
  else if(pin == 57 && !value) PORTF &= ~(1 << (3));
  else if(pin == 58 && value) PORTF  |= (1 << (4));
  else if(pin == 58 && !value) PORTF &= ~(1 << (4));
  else if(pin == 59 && value) PORTF  |= (1 << (5));
  else if(pin == 59 && !value) PORTF &= ~(1 << (5));
  else if(pin == 60 && value) PORTF  |= (1 << (6));
  else if(pin == 60 && !value) PORTF &= ~(1 << (6));
  else if(pin == 61 && value) PORTF  |= (1 << (7));
  else if(pin == 61 && !value) PORTF &= ~(1 << (7));
  else if(pin == 62 && value) PORTK  |= (1 << (0));
  else if(pin == 62 && !value) PORTK &= ~(1 << (0));
  else if(pin == 63 && value) PORTK  |= (1 << (1));
  else if(pin == 63 && !value) PORTK &= ~(1 << (1));
  else if(pin == 64 && value) PORTK  |= (1 << (2));
  else if(pin == 64 && !value) PORTK &= ~(1 << (2));
  else if(pin == 65 && value) PORTK  |= (1 << (3));
  else if(pin == 65 && !value) PORTK &= ~(1 << (3));
  else if(pin == 66 && value) PORTK  |= (1 << (4));
  else if(pin == 66 && !value) PORTK &= ~(1 << (4));
  else if(pin == 67 && value) PORTK  |= (1 << (5));
  else if(pin == 67 && !value) PORTK &= ~(1 << (5));
  else if(pin == 68 && value) PORTK  |= (1 << (6));
  else if(pin == 68 && !value) PORTK &= ~(1 << (6));
  else if(pin == 69 && value) PORTK  |= (1 << (7));
  else if(pin == 69 && !value) PORTK &= ~(1 << (7));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PINE & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PINE & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PINE & (1 << (4)) ? HIGH : LOW;
  else if(pin == 3) return PINE & (1 << (5)) ? HIGH : LOW;
  else if(pin == 4) return PING & (1 << (5)) ? HIGH : LOW;
  else if(pin == 5) return PINE & (1 << (3)) ? HIGH : LOW;
  else if(pin == 6) return PINH & (1 << (3)) ? HIGH : LOW;
  else if(pin == 7) return PINH & (1 << (4)) ? HIGH : LOW;
  else if(pin == 8) return PINH & (1 << (5)) ? HIGH : LOW;
  else if(pin == 9) return PINH & (1 << (6)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (7)) ? HIGH : LOW;
  else if(pin == 14) return PINJ & (1 << (1)) ? HIGH : LOW;
  else if(pin == 15) return PINJ & (1 << (0)) ? HIGH : LOW;
  else if(pin == 16) return PINH & (1 << (1)) ? HIGH : LOW;
  else if(pin == 17) return PINH & (1 << (0)) ? HIGH : LOW;
  else if(pin == 18) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 19) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 20) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 21) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 22) return PINA & (1 << (0)) ? HIGH : LOW;
  else if(pin == 23) return PINA & (1 << (1)) ? HIGH : LOW;
  else if(pin == 24) return PINA & (1 << (2)) ? HIGH : LOW;
  else if(pin == 25) return PINA & (1 << (3)) ? HIGH : LOW;
  else if(pin == 26) return PINA & (1 << (4)) ? HIGH : LOW;
  else if(pin == 27) return PINA & (1 << (5)) ? HIGH : LOW;
  else if(pin == 28) return PINA & (1 << (6)) ? HIGH : LOW;
  else if(pin == 29) return PINA & (1 << (7)) ? HIGH : LOW;
  else if(pin == 30) return PINC & (1 << (7)) ? HIGH : LOW;
  else if(pin == 31) return PINC & (1 << (6)) ? HIGH : LOW;
  else if(pin == 32) return PINC & (1 << (5)) ? HIGH : LOW;
  else if(pin == 33) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 34) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 35) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 36) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 37) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 38) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 39) return PING & (1 << (2)) ? HIGH : LOW;
  else if(pin == 40) return PING & (1 << (1)) ? HIGH : LOW;
  else if(pin == 41) return PING & (1 << (0)) ? HIGH : LOW;
  else if(pin == 42) return PINL & (1 << (7)) ? HIGH : LOW;
  else if(pin == 43) return PINL & (1 << (6)) ? HIGH : LOW;
  else if(pin == 44) return PINL & (1 << (5)) ? HIGH : LOW;
  else if(pin == 45) return PINL & (1 << (4)) ? HIGH : LOW;
  else if(pin == 46) return PINL & (1 << (3)) ? HIGH : LOW;
  else if(pin == 47) return PINL & (1 << (2)) ? HIGH : LOW;
  else if(pin == 48) return PINL & (1 << (1)) ? HIGH : LOW;
  else if(pin == 49) return PINL & (1 << (0)) ? HIGH : LOW;
  else if(pin == 50) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 51) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 52) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 53) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 54) return PINF & (1 << (0)) ? HIGH : LOW;
  else if(pin == 55) return PINF & (1 << (1)) ? HIGH : LOW;
  else if(pin == 56) return PINF & (1 << (2)) ? HIGH : LOW;
  else if(pin == 57) return PINF & (1 << (3)) ? HIGH : LOW;
  else if(pin == 58) return PINF & (1 << (4)) ? HIGH : LOW;
  else if(pin == 59) return PINF & (1 << (5)) ? HIGH : LOW;
  else if(pin == 60) return PINF & (1 << (6)) ? HIGH : LOW;
  else if(pin == 61) return PINF & (1 << (7)) ? HIGH : LOW;
  else if(pin == 62) return PINK & (1 << (0)) ? HIGH : LOW;
  else if(pin == 63) return PINK & (1 << (1)) ? HIGH : LOW;
  else if(pin == 64) return PINK & (1 << (2)) ? HIGH : LOW;
  else if(pin == 65) return PINK & (1 << (3)) ? HIGH : LOW;
  else if(pin == 66) return PINK & (1 << (4)) ? HIGH : LOW;
  else if(pin == 67) return PINK & (1 << (5)) ? HIGH : LOW;
  else if(pin == 68) return PINK & (1 << (6)) ? HIGH : LOW;
  else if(pin == 69) return PINK & (1 << (7)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 2) TCCR3A &= ~COM3B1;
  else if(pin == 3) TCCR3A &= ~COM3C1;
  else if(pin == 4) TCCR0A &= ~COM0B1;
  else if(pin == 5) TCCR3A &= ~COM3A1;
  else if(pin == 6) TCCR4A &= ~COM4A1;
  else if(pin == 7) TCCR4A &= ~COM4B1;
  else if(pin == 8) TCCR4A &= ~COM4C1;
  else if(pin == 9) TCCR2A &= ~COM2B1;
  else if(pin == 10) TCCR2A &= ~COM2A1;
  else if(pin == 11) TCCR1A &= ~COM1A1;
  else if(pin == 12) TCCR1A &= ~COM1B1;
  else if(pin == 13) TCCR0A &= ~COM0A1;
  else if(pin == 44) TCCR5A &= ~COM5C1;
  else if(pin == 45) TCCR5A &= ~COM5B1;
  else if(pin == 46) TCCR5A &= ~COM5A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   pro5v328 | atmega328 | ethernet | uno
 *   Arduino Pro or Pro Mini (5V, 16 MHz) w/ ATmega328 | Arduino Duemilanove w/ ATmega328 | Arduino Ethernet | Arduino Uno
 *   MCU: atmega328p
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 6 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR2A &= ~COM2B1;
  else if(pin == 5) TCCR0A &= ~COM0B1;
  else if(pin == 6) TCCR0A &= ~COM0A1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2A &= ~COM2A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   bt328 | nano328 | mini328
 *   Arduino BT w/ ATmega328 | Arduino Nano w/ ATmega328 | Arduino Mini w/ ATmega328
 *   MCU: atmega328p
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 8 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 0) DDRD &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 1) DDRD &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 2) DDRD &= ~(1 << (2));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 3) DDRD &= ~(1 << (3));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRD |= (1 << (5));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (5));
    PORTD &= ~(1 << (5));
  } else if(pin == 5) DDRD &= ~(1 << (5));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 6) DDRD &= ~(1 << (6));
  else if(pin == 7 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 7) DDRD &= ~(1 << (7));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 8) DDRB &= ~(1 << (0));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 9) DDRB &= ~(1 << (1));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 10) DDRB &= ~(1 << (2));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 11) DDRB &= ~(1 << (3));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 12) DDRB &= ~(1 << (4));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 13) DDRB &= ~(1 << (5));
  else if(pin == 14 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 14) DDRC &= ~(1 << (0));
  else if(pin == 15 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 15) DDRC &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 16) DDRC &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 17) DDRC &= ~(1 << (3));
  else if(pin == 18 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 18) DDRC &= ~(1 << (4));
  else if(pin == 19 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 19) DDRC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (0));
  else if(pin == 0 && !value) PORTD &= ~(1 << (0));
  else if(pin == 1 && value) PORTD  |= (1 << (1));
  else if(pin == 1 && !value) PORTD &= ~(1 << (1));
  else if(pin == 2 && value) PORTD  |= (1 << (2));
  else if(pin == 2 && !value) PORTD &= ~(1 << (2));
  else if(pin == 3 && value) PORTD  |= (1 << (3));
  else if(pin == 3 && !value) PORTD &= ~(1 << (3));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTD  |= (1 << (5));
  else if(pin == 5 && !value) PORTD &= ~(1 << (5));
  else if(pin == 6 && value) PORTD  |= (1 << (6));
  else if(pin == 6 && !value) PORTD &= ~(1 << (6));
  else if(pin == 7 && value) PORTD  |= (1 << (7));
  else if(pin == 7 && !value) PORTD &= ~(1 << (7));
  else if(pin == 8 && value) PORTB  |= (1 << (0));
  else if(pin == 8 && !value) PORTB &= ~(1 << (0));
  else if(pin == 9 && value) PORTB  |= (1 << (1));
  else if(pin == 9 && !value) PORTB &= ~(1 << (1));
  else if(pin == 10 && value) PORTB  |= (1 << (2));
  else if(pin == 10 && !value) PORTB &= ~(1 << (2));
  else if(pin == 11 && value) PORTB  |= (1 << (3));
  else if(pin == 11 && !value) PORTB &= ~(1 << (3));
  else if(pin == 12 && value) PORTB  |= (1 << (4));
  else if(pin == 12 && !value) PORTB &= ~(1 << (4));
  else if(pin == 13 && value) PORTB  |= (1 << (5));
  else if(pin == 13 && !value) PORTB &= ~(1 << (5));
  else if(pin == 14 && value) PORTC  |= (1 << (0));
  else if(pin == 14 && !value) PORTC &= ~(1 << (0));
  else if(pin == 15 && value) PORTC  |= (1 << (1));
  else if(pin == 15 && !value) PORTC &= ~(1 << (1));
  else if(pin == 16 && value) PORTC  |= (1 << (2));
  else if(pin == 16 && !value) PORTC &= ~(1 << (2));
  else if(pin == 17 && value) PORTC  |= (1 << (3));
  else if(pin == 17 && !value) PORTC &= ~(1 << (3));
  else if(pin == 18 && value) PORTC  |= (1 << (4));
  else if(pin == 18 && !value) PORTC &= ~(1 << (4));
  else if(pin == 19 && value) PORTC  |= (1 << (5));
  else if(pin == 19 && !value) PORTC &= ~(1 << (5));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PIND & (1 << (5)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 7) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 14) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 15) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 18) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 19) return PINC & (1 << (5)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR2A &= ~COM2B1;
  else if(pin == 5) TCCR0A &= ~COM0B1;
  else if(pin == 6) TCCR0A &= ~COM0A1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR2A &= ~COM2A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   esplora
 *   Arduino Esplora
 *   MCU: atmega32u4
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 12 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && defined(USB_PID) && (USB_PID+0) == 0x803C
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 0) DDRD &= ~(1 << (2));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 1) DDRD &= ~(1 << (3));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 2) DDRD &= ~(1 << (1));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 3) DDRD &= ~(1 << (0));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRC |= (1 << (6));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (6));
    PORTC &= ~(1 << (6));
  } else if(pin == 5) DDRC &= ~(1 << (6));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 6) DDRD &= ~(1 << (7));
  else if(pin == 7 && mode == INPUT) DDRE |= (1 << (6));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (6));
    PORTE &= ~(1 << (6));
  } else if(pin == 7) DDRE &= ~(1 << (6));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 8) DDRB &= ~(1 << (4));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 9) DDRB &= ~(1 << (5));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 10) DDRB &= ~(1 << (6));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (7));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (7));
    PORTB &= ~(1 << (7));
  } else if(pin == 11) DDRB &= ~(1 << (7));
  else if(pin == 12 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 12) DDRD &= ~(1 << (6));
  else if(pin == 13 && mode == INPUT) DDRC |= (1 << (7));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (7));
    PORTC &= ~(1 << (7));
  } else if(pin == 13) DDRC &= ~(1 << (7));
  else if(pin == 14 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 14) DDRB &= ~(1 << (3));
  else if(pin == 15 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 15) DDRB &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 16) DDRB &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 17) DDRB &= ~(1 << (0));
  else if(pin == 18 && mode == INPUT) DDRF |= (1 << (7));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (7));
    PORTF &= ~(1 << (7));
  } else if(pin == 18) DDRF &= ~(1 << (7));
  else if(pin == 19 && mode == INPUT) DDRF |= (1 << (6));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (6));
    PORTF &= ~(1 << (6));
  } else if(pin == 19) DDRF &= ~(1 << (6));
  else if(pin == 20 && mode == INPUT) DDRF |= (1 << (5));
  else if(pin == 20 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (5));
    PORTF &= ~(1 << (5));
  } else if(pin == 20) DDRF &= ~(1 << (5));
  else if(pin == 21 && mode == INPUT) DDRF |= (1 << (4));
  else if(pin == 21 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (4));
    PORTF &= ~(1 << (4));
  } else if(pin == 21) DDRF &= ~(1 << (4));
  else if(pin == 22 && mode == INPUT) DDRF |= (1 << (1));
  else if(pin == 22 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (1));
    PORTF &= ~(1 << (1));
  } else if(pin == 22) DDRF &= ~(1 << (1));
  else if(pin == 23 && mode == INPUT) DDRF |= (1 << (0));
  else if(pin == 23 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (0));
    PORTF &= ~(1 << (0));
  } else if(pin == 23) DDRF &= ~(1 << (0));
  else if(pin == 24 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 24 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 24) DDRD &= ~(1 << (4));
  else if(pin == 25 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 25 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 25) DDRD &= ~(1 << (7));
  else if(pin == 26 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 26 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 26) DDRB &= ~(1 << (4));
  else if(pin == 27 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 27 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 27) DDRB &= ~(1 << (5));
  else if(pin == 28 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 28 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 28) DDRB &= ~(1 << (6));
  else if(pin == 29 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 29 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 29) DDRD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (2));
  else if(pin == 0 && !value) PORTD &= ~(1 << (2));
  else if(pin == 1 && value) PORTD  |= (1 << (3));
  else if(pin == 1 && !value) PORTD &= ~(1 << (3));
  else if(pin == 2 && value) PORTD  |= (1 << (1));
  else if(pin == 2 && !value) PORTD &= ~(1 << (1));
  else if(pin == 3 && value) PORTD  |= (1 << (0));
  else if(pin == 3 && !value) PORTD &= ~(1 << (0));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTC  |= (1 << (6));
  else if(pin == 5 && !value) PORTC &= ~(1 << (6));
  else if(pin == 6 && value) PORTD  |= (1 << (7));
  else if(pin == 6 && !value) PORTD &= ~(1 << (7));
  else if(pin == 7 && value) PORTE  |= (1 << (6));
  else if(pin == 7 && !value) PORTE &= ~(1 << (6));
  else if(pin == 8 && value) PORTB  |= (1 << (4));
  else if(pin == 8 && !value) PORTB &= ~(1 << (4));
  else if(pin == 9 && value) PORTB  |= (1 << (5));
  else if(pin == 9 && !value) PORTB &= ~(1 << (5));
  else if(pin == 10 && value) PORTB  |= (1 << (6));
  else if(pin == 10 && !value) PORTB &= ~(1 << (6));
  else if(pin == 11 && value) PORTB  |= (1 << (7));
  else if(pin == 11 && !value) PORTB &= ~(1 << (7));
  else if(pin == 12 && value) PORTD  |= (1 << (6));
  else if(pin == 12 && !value) PORTD &= ~(1 << (6));
  else if(pin == 13 && value) PORTC  |= (1 << (7));
  else if(pin == 13 && !value) PORTC &= ~(1 << (7));
  else if(pin == 14 && value) PORTB  |= (1 << (3));
  else if(pin == 14 && !value) PORTB &= ~(1 << (3));
  else if(pin == 15 && value) PORTB  |= (1 << (1));
  else if(pin == 15 && !value) PORTB &= ~(1 << (1));
  else if(pin == 16 && value) PORTB  |= (1 << (2));
  else if(pin == 16 && !value) PORTB &= ~(1 << (2));
  else if(pin == 17 && value) PORTB  |= (1 << (0));
  else if(pin == 17 && !value) PORTB &= ~(1 << (0));
  else if(pin == 18 && value) PORTF  |= (1 << (7));
  else if(pin == 18 && !value) PORTF &= ~(1 << (7));
  else if(pin == 19 && value) PORTF  |= (1 << (6));
  else if(pin == 19 && !value) PORTF &= ~(1 << (6));
  else if(pin == 20 && value) PORTF  |= (1 << (5));
  else if(pin == 20 && !value) PORTF &= ~(1 << (5));
  else if(pin == 21 && value) PORTF  |= (1 << (4));
  else if(pin == 21 && !value) PORTF &= ~(1 << (4));
  else if(pin == 22 && value) PORTF  |= (1 << (1));
  else if(pin == 22 && !value) PORTF &= ~(1 << (1));
  else if(pin == 23 && value) PORTF  |= (1 << (0));
  else if(pin == 23 && !value) PORTF &= ~(1 << (0));
  else if(pin == 24 && value) PORTD  |= (1 << (4));
  else if(pin == 24 && !value) PORTD &= ~(1 << (4));
  else if(pin == 25 && value) PORTD  |= (1 << (7));
  else if(pin == 25 && !value) PORTD &= ~(1 << (7));
  else if(pin == 26 && value) PORTB  |= (1 << (4));
  else if(pin == 26 && !value) PORTB &= ~(1 << (4));
  else if(pin == 27 && value) PORTB  |= (1 << (5));
  else if(pin == 27 && !value) PORTB &= ~(1 << (5));
  else if(pin == 28 && value) PORTB  |= (1 << (6));
  else if(pin == 28 && !value) PORTB &= ~(1 << (6));
  else if(pin == 29 && value) PORTD  |= (1 << (6));
  else if(pin == 29 && !value) PORTD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PINC & (1 << (6)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 7) return PINE & (1 << (6)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (7)) ? HIGH : LOW;
  else if(pin == 12) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 13) return PINC & (1 << (7)) ? HIGH : LOW;
  else if(pin == 14) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 15) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 18) return PINF & (1 << (7)) ? HIGH : LOW;
  else if(pin == 19) return PINF & (1 << (6)) ? HIGH : LOW;
  else if(pin == 20) return PINF & (1 << (5)) ? HIGH : LOW;
  else if(pin == 21) return PINF & (1 << (4)) ? HIGH : LOW;
  else if(pin == 22) return PINF & (1 << (1)) ? HIGH : LOW;
  else if(pin == 23) return PINF & (1 << (0)) ? HIGH : LOW;
  else if(pin == 24) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 25) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 26) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 27) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 28) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 29) return PIND & (1 << (6)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR0A &= ~COM0B1;
  else if(pin == 5) TCCR3A &= ~COM3A1;
  else if(pin == 6) TCCR4C &= ~COM4D1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR0A &= ~COM0A1;
  else if(pin == 13) TCCR4A &= ~COM4A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   mega
 *   Arduino Mega (ATmega1280)
 *   MCU: atmega1280
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 16 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x97 && (!defined(USB_PID) || !(USB_PID+0))
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRE |= (1 << (0));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (0));
    PORTE &= ~(1 << (0));
  } else if(pin == 0) DDRE &= ~(1 << (0));
  else if(pin == 1 && mode == INPUT) DDRE |= (1 << (1));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (1));
    PORTE &= ~(1 << (1));
  } else if(pin == 1) DDRE &= ~(1 << (1));
  else if(pin == 2 && mode == INPUT) DDRE |= (1 << (4));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (4));
    PORTE &= ~(1 << (4));
  } else if(pin == 2) DDRE &= ~(1 << (4));
  else if(pin == 3 && mode == INPUT) DDRE |= (1 << (5));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (5));
    PORTE &= ~(1 << (5));
  } else if(pin == 3) DDRE &= ~(1 << (5));
  else if(pin == 4 && mode == INPUT) DDRG |= (1 << (5));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (5));
    PORTG &= ~(1 << (5));
  } else if(pin == 4) DDRG &= ~(1 << (5));
  else if(pin == 5 && mode == INPUT) DDRE |= (1 << (3));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (3));
    PORTE &= ~(1 << (3));
  } else if(pin == 5) DDRE &= ~(1 << (3));
  else if(pin == 6 && mode == INPUT) DDRH |= (1 << (3));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (3));
    PORTH &= ~(1 << (3));
  } else if(pin == 6) DDRH &= ~(1 << (3));
  else if(pin == 7 && mode == INPUT) DDRH |= (1 << (4));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (4));
    PORTH &= ~(1 << (4));
  } else if(pin == 7) DDRH &= ~(1 << (4));
  else if(pin == 8 && mode == INPUT) DDRH |= (1 << (5));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (5));
    PORTH &= ~(1 << (5));
  } else if(pin == 8) DDRH &= ~(1 << (5));
  else if(pin == 9 && mode == INPUT) DDRH |= (1 << (6));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (6));
    PORTH &= ~(1 << (6));
  } else if(pin == 9) DDRH &= ~(1 << (6));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 10) DDRB &= ~(1 << (4));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 11) DDRB &= ~(1 << (5));
  else if(pin == 12 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 12) DDRB &= ~(1 << (6));
  else if(pin == 13 && mode == INPUT) DDRB |= (1 << (7));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (7));
    PORTB &= ~(1 << (7));
  } else if(pin == 13) DDRB &= ~(1 << (7));
  else if(pin == 14 && mode == INPUT) DDRJ |= (1 << (1));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRJ |= (1 << (1));
    PORTJ &= ~(1 << (1));
  } else if(pin == 14) DDRJ &= ~(1 << (1));
  else if(pin == 15 && mode == INPUT) DDRJ |= (1 << (0));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRJ |= (1 << (0));
    PORTJ &= ~(1 << (0));
  } else if(pin == 15) DDRJ &= ~(1 << (0));
  else if(pin == 16 && mode == INPUT) DDRH |= (1 << (1));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (1));
    PORTH &= ~(1 << (1));
  } else if(pin == 16) DDRH &= ~(1 << (1));
  else if(pin == 17 && mode == INPUT) DDRH |= (1 << (0));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRH |= (1 << (0));
    PORTH &= ~(1 << (0));
  } else if(pin == 17) DDRH &= ~(1 << (0));
  else if(pin == 18 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 18) DDRD &= ~(1 << (3));
  else if(pin == 19 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 19) DDRD &= ~(1 << (2));
  else if(pin == 20 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 20 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 20) DDRD &= ~(1 << (1));
  else if(pin == 21 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 21 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 21) DDRD &= ~(1 << (0));
  else if(pin == 22 && mode == INPUT) DDRA |= (1 << (0));
  else if(pin == 22 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (0));
    PORTA &= ~(1 << (0));
  } else if(pin == 22) DDRA &= ~(1 << (0));
  else if(pin == 23 && mode == INPUT) DDRA |= (1 << (1));
  else if(pin == 23 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (1));
    PORTA &= ~(1 << (1));
  } else if(pin == 23) DDRA &= ~(1 << (1));
  else if(pin == 24 && mode == INPUT) DDRA |= (1 << (2));
  else if(pin == 24 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (2));
    PORTA &= ~(1 << (2));
  } else if(pin == 24) DDRA &= ~(1 << (2));
  else if(pin == 25 && mode == INPUT) DDRA |= (1 << (3));
  else if(pin == 25 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (3));
    PORTA &= ~(1 << (3));
  } else if(pin == 25) DDRA &= ~(1 << (3));
  else if(pin == 26 && mode == INPUT) DDRA |= (1 << (4));
  else if(pin == 26 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (4));
    PORTA &= ~(1 << (4));
  } else if(pin == 26) DDRA &= ~(1 << (4));
  else if(pin == 27 && mode == INPUT) DDRA |= (1 << (5));
  else if(pin == 27 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (5));
    PORTA &= ~(1 << (5));
  } else if(pin == 27) DDRA &= ~(1 << (5));
  else if(pin == 28 && mode == INPUT) DDRA |= (1 << (6));
  else if(pin == 28 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (6));
    PORTA &= ~(1 << (6));
  } else if(pin == 28) DDRA &= ~(1 << (6));
  else if(pin == 29 && mode == INPUT) DDRA |= (1 << (7));
  else if(pin == 29 && mode == INPUT_PULLUP) {
    DDRA |= (1 << (7));
    PORTA &= ~(1 << (7));
  } else if(pin == 29) DDRA &= ~(1 << (7));
  else if(pin == 30 && mode == INPUT) DDRC |= (1 << (7));
  else if(pin == 30 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (7));
    PORTC &= ~(1 << (7));
  } else if(pin == 30) DDRC &= ~(1 << (7));
  else if(pin == 31 && mode == INPUT) DDRC |= (1 << (6));
  else if(pin == 31 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (6));
    PORTC &= ~(1 << (6));
  } else if(pin == 31) DDRC &= ~(1 << (6));
  else if(pin == 32 && mode == INPUT) DDRC |= (1 << (5));
  else if(pin == 32 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (5));
    PORTC &= ~(1 << (5));
  } else if(pin == 32) DDRC &= ~(1 << (5));
  else if(pin == 33 && mode == INPUT) DDRC |= (1 << (4));
  else if(pin == 33 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (4));
    PORTC &= ~(1 << (4));
  } else if(pin == 33) DDRC &= ~(1 << (4));
  else if(pin == 34 && mode == INPUT) DDRC |= (1 << (3));
  else if(pin == 34 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (3));
    PORTC &= ~(1 << (3));
  } else if(pin == 34) DDRC &= ~(1 << (3));
  else if(pin == 35 && mode == INPUT) DDRC |= (1 << (2));
  else if(pin == 35 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (2));
    PORTC &= ~(1 << (2));
  } else if(pin == 35) DDRC &= ~(1 << (2));
  else if(pin == 36 && mode == INPUT) DDRC |= (1 << (1));
  else if(pin == 36 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (1));
    PORTC &= ~(1 << (1));
  } else if(pin == 36) DDRC &= ~(1 << (1));
  else if(pin == 37 && mode == INPUT) DDRC |= (1 << (0));
  else if(pin == 37 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (0));
    PORTC &= ~(1 << (0));
  } else if(pin == 37) DDRC &= ~(1 << (0));
  else if(pin == 38 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 38 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 38) DDRD &= ~(1 << (7));
  else if(pin == 39 && mode == INPUT) DDRG |= (1 << (2));
  else if(pin == 39 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (2));
    PORTG &= ~(1 << (2));
  } else if(pin == 39) DDRG &= ~(1 << (2));
  else if(pin == 40 && mode == INPUT) DDRG |= (1 << (1));
  else if(pin == 40 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (1));
    PORTG &= ~(1 << (1));
  } else if(pin == 40) DDRG &= ~(1 << (1));
  else if(pin == 41 && mode == INPUT) DDRG |= (1 << (0));
  else if(pin == 41 && mode == INPUT_PULLUP) {
    DDRG |= (1 << (0));
    PORTG &= ~(1 << (0));
  } else if(pin == 41) DDRG &= ~(1 << (0));
  else if(pin == 42 && mode == INPUT) DDRL |= (1 << (7));
  else if(pin == 42 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (7));
    PORTL &= ~(1 << (7));
  } else if(pin == 42) DDRL &= ~(1 << (7));
  else if(pin == 43 && mode == INPUT) DDRL |= (1 << (6));
  else if(pin == 43 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (6));
    PORTL &= ~(1 << (6));
  } else if(pin == 43) DDRL &= ~(1 << (6));
  else if(pin == 44 && mode == INPUT) DDRL |= (1 << (5));
  else if(pin == 44 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (5));
    PORTL &= ~(1 << (5));
  } else if(pin == 44) DDRL &= ~(1 << (5));
  else if(pin == 45 && mode == INPUT) DDRL |= (1 << (4));
  else if(pin == 45 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (4));
    PORTL &= ~(1 << (4));
  } else if(pin == 45) DDRL &= ~(1 << (4));
  else if(pin == 46 && mode == INPUT) DDRL |= (1 << (3));
  else if(pin == 46 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (3));
    PORTL &= ~(1 << (3));
  } else if(pin == 46) DDRL &= ~(1 << (3));
  else if(pin == 47 && mode == INPUT) DDRL |= (1 << (2));
  else if(pin == 47 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (2));
    PORTL &= ~(1 << (2));
  } else if(pin == 47) DDRL &= ~(1 << (2));
  else if(pin == 48 && mode == INPUT) DDRL |= (1 << (1));
  else if(pin == 48 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (1));
    PORTL &= ~(1 << (1));
  } else if(pin == 48) DDRL &= ~(1 << (1));
  else if(pin == 49 && mode == INPUT) DDRL |= (1 << (0));
  else if(pin == 49 && mode == INPUT_PULLUP) {
    DDRL |= (1 << (0));
    PORTL &= ~(1 << (0));
  } else if(pin == 49) DDRL &= ~(1 << (0));
  else if(pin == 50 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 50 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 50) DDRB &= ~(1 << (3));
  else if(pin == 51 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 51 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 51) DDRB &= ~(1 << (2));
  else if(pin == 52 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 52 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 52) DDRB &= ~(1 << (1));
  else if(pin == 53 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 53 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 53) DDRB &= ~(1 << (0));
  else if(pin == 54 && mode == INPUT) DDRF |= (1 << (0));
  else if(pin == 54 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (0));
    PORTF &= ~(1 << (0));
  } else if(pin == 54) DDRF &= ~(1 << (0));
  else if(pin == 55 && mode == INPUT) DDRF |= (1 << (1));
  else if(pin == 55 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (1));
    PORTF &= ~(1 << (1));
  } else if(pin == 55) DDRF &= ~(1 << (1));
  else if(pin == 56 && mode == INPUT) DDRF |= (1 << (2));
  else if(pin == 56 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (2));
    PORTF &= ~(1 << (2));
  } else if(pin == 56) DDRF &= ~(1 << (2));
  else if(pin == 57 && mode == INPUT) DDRF |= (1 << (3));
  else if(pin == 57 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (3));
    PORTF &= ~(1 << (3));
  } else if(pin == 57) DDRF &= ~(1 << (3));
  else if(pin == 58 && mode == INPUT) DDRF |= (1 << (4));
  else if(pin == 58 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (4));
    PORTF &= ~(1 << (4));
  } else if(pin == 58) DDRF &= ~(1 << (4));
  else if(pin == 59 && mode == INPUT) DDRF |= (1 << (5));
  else if(pin == 59 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (5));
    PORTF &= ~(1 << (5));
  } else if(pin == 59) DDRF &= ~(1 << (5));
  else if(pin == 60 && mode == INPUT) DDRF |= (1 << (6));
  else if(pin == 60 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (6));
    PORTF &= ~(1 << (6));
  } else if(pin == 60) DDRF &= ~(1 << (6));
  else if(pin == 61 && mode == INPUT) DDRF |= (1 << (7));
  else if(pin == 61 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (7));
    PORTF &= ~(1 << (7));
  } else if(pin == 61) DDRF &= ~(1 << (7));
  else if(pin == 62 && mode == INPUT) DDRK |= (1 << (0));
  else if(pin == 62 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (0));
    PORTK &= ~(1 << (0));
  } else if(pin == 62) DDRK &= ~(1 << (0));
  else if(pin == 63 && mode == INPUT) DDRK |= (1 << (1));
  else if(pin == 63 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (1));
    PORTK &= ~(1 << (1));
  } else if(pin == 63) DDRK &= ~(1 << (1));
  else if(pin == 64 && mode == INPUT) DDRK |= (1 << (2));
  else if(pin == 64 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (2));
    PORTK &= ~(1 << (2));
  } else if(pin == 64) DDRK &= ~(1 << (2));
  else if(pin == 65 && mode == INPUT) DDRK |= (1 << (3));
  else if(pin == 65 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (3));
    PORTK &= ~(1 << (3));
  } else if(pin == 65) DDRK &= ~(1 << (3));
  else if(pin == 66 && mode == INPUT) DDRK |= (1 << (4));
  else if(pin == 66 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (4));
    PORTK &= ~(1 << (4));
  } else if(pin == 66) DDRK &= ~(1 << (4));
  else if(pin == 67 && mode == INPUT) DDRK |= (1 << (5));
  else if(pin == 67 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (5));
    PORTK &= ~(1 << (5));
  } else if(pin == 67) DDRK &= ~(1 << (5));
  else if(pin == 68 && mode == INPUT) DDRK |= (1 << (6));
  else if(pin == 68 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (6));
    PORTK &= ~(1 << (6));
  } else if(pin == 68) DDRK &= ~(1 << (6));
  else if(pin == 69 && mode == INPUT) DDRK |= (1 << (7));
  else if(pin == 69 && mode == INPUT_PULLUP) {
    DDRK |= (1 << (7));
    PORTK &= ~(1 << (7));
  } else if(pin == 69) DDRK &= ~(1 << (7));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTE  |= (1 << (0));
  else if(pin == 0 && !value) PORTE &= ~(1 << (0));
  else if(pin == 1 && value) PORTE  |= (1 << (1));
  else if(pin == 1 && !value) PORTE &= ~(1 << (1));
  else if(pin == 2 && value) PORTE  |= (1 << (4));
  else if(pin == 2 && !value) PORTE &= ~(1 << (4));
  else if(pin == 3 && value) PORTE  |= (1 << (5));
  else if(pin == 3 && !value) PORTE &= ~(1 << (5));
  else if(pin == 4 && value) PORTG  |= (1 << (5));
  else if(pin == 4 && !value) PORTG &= ~(1 << (5));
  else if(pin == 5 && value) PORTE  |= (1 << (3));
  else if(pin == 5 && !value) PORTE &= ~(1 << (3));
  else if(pin == 6 && value) PORTH  |= (1 << (3));
  else if(pin == 6 && !value) PORTH &= ~(1 << (3));
  else if(pin == 7 && value) PORTH  |= (1 << (4));
  else if(pin == 7 && !value) PORTH &= ~(1 << (4));
  else if(pin == 8 && value) PORTH  |= (1 << (5));
  else if(pin == 8 && !value) PORTH &= ~(1 << (5));
  else if(pin == 9 && value) PORTH  |= (1 << (6));
  else if(pin == 9 && !value) PORTH &= ~(1 << (6));
  else if(pin == 10 && value) PORTB  |= (1 << (4));
  else if(pin == 10 && !value) PORTB &= ~(1 << (4));
  else if(pin == 11 && value) PORTB  |= (1 << (5));
  else if(pin == 11 && !value) PORTB &= ~(1 << (5));
  else if(pin == 12 && value) PORTB  |= (1 << (6));
  else if(pin == 12 && !value) PORTB &= ~(1 << (6));
  else if(pin == 13 && value) PORTB  |= (1 << (7));
  else if(pin == 13 && !value) PORTB &= ~(1 << (7));
  else if(pin == 14 && value) PORTJ  |= (1 << (1));
  else if(pin == 14 && !value) PORTJ &= ~(1 << (1));
  else if(pin == 15 && value) PORTJ  |= (1 << (0));
  else if(pin == 15 && !value) PORTJ &= ~(1 << (0));
  else if(pin == 16 && value) PORTH  |= (1 << (1));
  else if(pin == 16 && !value) PORTH &= ~(1 << (1));
  else if(pin == 17 && value) PORTH  |= (1 << (0));
  else if(pin == 17 && !value) PORTH &= ~(1 << (0));
  else if(pin == 18 && value) PORTD  |= (1 << (3));
  else if(pin == 18 && !value) PORTD &= ~(1 << (3));
  else if(pin == 19 && value) PORTD  |= (1 << (2));
  else if(pin == 19 && !value) PORTD &= ~(1 << (2));
  else if(pin == 20 && value) PORTD  |= (1 << (1));
  else if(pin == 20 && !value) PORTD &= ~(1 << (1));
  else if(pin == 21 && value) PORTD  |= (1 << (0));
  else if(pin == 21 && !value) PORTD &= ~(1 << (0));
  else if(pin == 22 && value) PORTA  |= (1 << (0));
  else if(pin == 22 && !value) PORTA &= ~(1 << (0));
  else if(pin == 23 && value) PORTA  |= (1 << (1));
  else if(pin == 23 && !value) PORTA &= ~(1 << (1));
  else if(pin == 24 && value) PORTA  |= (1 << (2));
  else if(pin == 24 && !value) PORTA &= ~(1 << (2));
  else if(pin == 25 && value) PORTA  |= (1 << (3));
  else if(pin == 25 && !value) PORTA &= ~(1 << (3));
  else if(pin == 26 && value) PORTA  |= (1 << (4));
  else if(pin == 26 && !value) PORTA &= ~(1 << (4));
  else if(pin == 27 && value) PORTA  |= (1 << (5));
  else if(pin == 27 && !value) PORTA &= ~(1 << (5));
  else if(pin == 28 && value) PORTA  |= (1 << (6));
  else if(pin == 28 && !value) PORTA &= ~(1 << (6));
  else if(pin == 29 && value) PORTA  |= (1 << (7));
  else if(pin == 29 && !value) PORTA &= ~(1 << (7));
  else if(pin == 30 && value) PORTC  |= (1 << (7));
  else if(pin == 30 && !value) PORTC &= ~(1 << (7));
  else if(pin == 31 && value) PORTC  |= (1 << (6));
  else if(pin == 31 && !value) PORTC &= ~(1 << (6));
  else if(pin == 32 && value) PORTC  |= (1 << (5));
  else if(pin == 32 && !value) PORTC &= ~(1 << (5));
  else if(pin == 33 && value) PORTC  |= (1 << (4));
  else if(pin == 33 && !value) PORTC &= ~(1 << (4));
  else if(pin == 34 && value) PORTC  |= (1 << (3));
  else if(pin == 34 && !value) PORTC &= ~(1 << (3));
  else if(pin == 35 && value) PORTC  |= (1 << (2));
  else if(pin == 35 && !value) PORTC &= ~(1 << (2));
  else if(pin == 36 && value) PORTC  |= (1 << (1));
  else if(pin == 36 && !value) PORTC &= ~(1 << (1));
  else if(pin == 37 && value) PORTC  |= (1 << (0));
  else if(pin == 37 && !value) PORTC &= ~(1 << (0));
  else if(pin == 38 && value) PORTD  |= (1 << (7));
  else if(pin == 38 && !value) PORTD &= ~(1 << (7));
  else if(pin == 39 && value) PORTG  |= (1 << (2));
  else if(pin == 39 && !value) PORTG &= ~(1 << (2));
  else if(pin == 40 && value) PORTG  |= (1 << (1));
  else if(pin == 40 && !value) PORTG &= ~(1 << (1));
  else if(pin == 41 && value) PORTG  |= (1 << (0));
  else if(pin == 41 && !value) PORTG &= ~(1 << (0));
  else if(pin == 42 && value) PORTL  |= (1 << (7));
  else if(pin == 42 && !value) PORTL &= ~(1 << (7));
  else if(pin == 43 && value) PORTL  |= (1 << (6));
  else if(pin == 43 && !value) PORTL &= ~(1 << (6));
  else if(pin == 44 && value) PORTL  |= (1 << (5));
  else if(pin == 44 && !value) PORTL &= ~(1 << (5));
  else if(pin == 45 && value) PORTL  |= (1 << (4));
  else if(pin == 45 && !value) PORTL &= ~(1 << (4));
  else if(pin == 46 && value) PORTL  |= (1 << (3));
  else if(pin == 46 && !value) PORTL &= ~(1 << (3));
  else if(pin == 47 && value) PORTL  |= (1 << (2));
  else if(pin == 47 && !value) PORTL &= ~(1 << (2));
  else if(pin == 48 && value) PORTL  |= (1 << (1));
  else if(pin == 48 && !value) PORTL &= ~(1 << (1));
  else if(pin == 49 && value) PORTL  |= (1 << (0));
  else if(pin == 49 && !value) PORTL &= ~(1 << (0));
  else if(pin == 50 && value) PORTB  |= (1 << (3));
  else if(pin == 50 && !value) PORTB &= ~(1 << (3));
  else if(pin == 51 && value) PORTB  |= (1 << (2));
  else if(pin == 51 && !value) PORTB &= ~(1 << (2));
  else if(pin == 52 && value) PORTB  |= (1 << (1));
  else if(pin == 52 && !value) PORTB &= ~(1 << (1));
  else if(pin == 53 && value) PORTB  |= (1 << (0));
  else if(pin == 53 && !value) PORTB &= ~(1 << (0));
  else if(pin == 54 && value) PORTF  |= (1 << (0));
  else if(pin == 54 && !value) PORTF &= ~(1 << (0));
  else if(pin == 55 && value) PORTF  |= (1 << (1));
  else if(pin == 55 && !value) PORTF &= ~(1 << (1));
  else if(pin == 56 && value) PORTF  |= (1 << (2));
  else if(pin == 56 && !value) PORTF &= ~(1 << (2));
  else if(pin == 57 && value) PORTF  |= (1 << (3));
  else if(pin == 57 && !value) PORTF &= ~(1 << (3));
  else if(pin == 58 && value) PORTF  |= (1 << (4));
  else if(pin == 58 && !value) PORTF &= ~(1 << (4));
  else if(pin == 59 && value) PORTF  |= (1 << (5));
  else if(pin == 59 && !value) PORTF &= ~(1 << (5));
  else if(pin == 60 && value) PORTF  |= (1 << (6));
  else if(pin == 60 && !value) PORTF &= ~(1 << (6));
  else if(pin == 61 && value) PORTF  |= (1 << (7));
  else if(pin == 61 && !value) PORTF &= ~(1 << (7));
  else if(pin == 62 && value) PORTK  |= (1 << (0));
  else if(pin == 62 && !value) PORTK &= ~(1 << (0));
  else if(pin == 63 && value) PORTK  |= (1 << (1));
  else if(pin == 63 && !value) PORTK &= ~(1 << (1));
  else if(pin == 64 && value) PORTK  |= (1 << (2));
  else if(pin == 64 && !value) PORTK &= ~(1 << (2));
  else if(pin == 65 && value) PORTK  |= (1 << (3));
  else if(pin == 65 && !value) PORTK &= ~(1 << (3));
  else if(pin == 66 && value) PORTK  |= (1 << (4));
  else if(pin == 66 && !value) PORTK &= ~(1 << (4));
  else if(pin == 67 && value) PORTK  |= (1 << (5));
  else if(pin == 67 && !value) PORTK &= ~(1 << (5));
  else if(pin == 68 && value) PORTK  |= (1 << (6));
  else if(pin == 68 && !value) PORTK &= ~(1 << (6));
  else if(pin == 69 && value) PORTK  |= (1 << (7));
  else if(pin == 69 && !value) PORTK &= ~(1 << (7));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PINE & (1 << (0)) ? HIGH : LOW;
  else if(pin == 1) return PINE & (1 << (1)) ? HIGH : LOW;
  else if(pin == 2) return PINE & (1 << (4)) ? HIGH : LOW;
  else if(pin == 3) return PINE & (1 << (5)) ? HIGH : LOW;
  else if(pin == 4) return PING & (1 << (5)) ? HIGH : LOW;
  else if(pin == 5) return PINE & (1 << (3)) ? HIGH : LOW;
  else if(pin == 6) return PINH & (1 << (3)) ? HIGH : LOW;
  else if(pin == 7) return PINH & (1 << (4)) ? HIGH : LOW;
  else if(pin == 8) return PINH & (1 << (5)) ? HIGH : LOW;
  else if(pin == 9) return PINH & (1 << (6)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 12) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 13) return PINB & (1 << (7)) ? HIGH : LOW;
  else if(pin == 14) return PINJ & (1 << (1)) ? HIGH : LOW;
  else if(pin == 15) return PINJ & (1 << (0)) ? HIGH : LOW;
  else if(pin == 16) return PINH & (1 << (1)) ? HIGH : LOW;
  else if(pin == 17) return PINH & (1 << (0)) ? HIGH : LOW;
  else if(pin == 18) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 19) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 20) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 21) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 22) return PINA & (1 << (0)) ? HIGH : LOW;
  else if(pin == 23) return PINA & (1 << (1)) ? HIGH : LOW;
  else if(pin == 24) return PINA & (1 << (2)) ? HIGH : LOW;
  else if(pin == 25) return PINA & (1 << (3)) ? HIGH : LOW;
  else if(pin == 26) return PINA & (1 << (4)) ? HIGH : LOW;
  else if(pin == 27) return PINA & (1 << (5)) ? HIGH : LOW;
  else if(pin == 28) return PINA & (1 << (6)) ? HIGH : LOW;
  else if(pin == 29) return PINA & (1 << (7)) ? HIGH : LOW;
  else if(pin == 30) return PINC & (1 << (7)) ? HIGH : LOW;
  else if(pin == 31) return PINC & (1 << (6)) ? HIGH : LOW;
  else if(pin == 32) return PINC & (1 << (5)) ? HIGH : LOW;
  else if(pin == 33) return PINC & (1 << (4)) ? HIGH : LOW;
  else if(pin == 34) return PINC & (1 << (3)) ? HIGH : LOW;
  else if(pin == 35) return PINC & (1 << (2)) ? HIGH : LOW;
  else if(pin == 36) return PINC & (1 << (1)) ? HIGH : LOW;
  else if(pin == 37) return PINC & (1 << (0)) ? HIGH : LOW;
  else if(pin == 38) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 39) return PING & (1 << (2)) ? HIGH : LOW;
  else if(pin == 40) return PING & (1 << (1)) ? HIGH : LOW;
  else if(pin == 41) return PING & (1 << (0)) ? HIGH : LOW;
  else if(pin == 42) return PINL & (1 << (7)) ? HIGH : LOW;
  else if(pin == 43) return PINL & (1 << (6)) ? HIGH : LOW;
  else if(pin == 44) return PINL & (1 << (5)) ? HIGH : LOW;
  else if(pin == 45) return PINL & (1 << (4)) ? HIGH : LOW;
  else if(pin == 46) return PINL & (1 << (3)) ? HIGH : LOW;
  else if(pin == 47) return PINL & (1 << (2)) ? HIGH : LOW;
  else if(pin == 48) return PINL & (1 << (1)) ? HIGH : LOW;
  else if(pin == 49) return PINL & (1 << (0)) ? HIGH : LOW;
  else if(pin == 50) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 51) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 52) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 53) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 54) return PINF & (1 << (0)) ? HIGH : LOW;
  else if(pin == 55) return PINF & (1 << (1)) ? HIGH : LOW;
  else if(pin == 56) return PINF & (1 << (2)) ? HIGH : LOW;
  else if(pin == 57) return PINF & (1 << (3)) ? HIGH : LOW;
  else if(pin == 58) return PINF & (1 << (4)) ? HIGH : LOW;
  else if(pin == 59) return PINF & (1 << (5)) ? HIGH : LOW;
  else if(pin == 60) return PINF & (1 << (6)) ? HIGH : LOW;
  else if(pin == 61) return PINF & (1 << (7)) ? HIGH : LOW;
  else if(pin == 62) return PINK & (1 << (0)) ? HIGH : LOW;
  else if(pin == 63) return PINK & (1 << (1)) ? HIGH : LOW;
  else if(pin == 64) return PINK & (1 << (2)) ? HIGH : LOW;
  else if(pin == 65) return PINK & (1 << (3)) ? HIGH : LOW;
  else if(pin == 66) return PINK & (1 << (4)) ? HIGH : LOW;
  else if(pin == 67) return PINK & (1 << (5)) ? HIGH : LOW;
  else if(pin == 68) return PINK & (1 << (6)) ? HIGH : LOW;
  else if(pin == 69) return PINK & (1 << (7)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 2) TCCR3A &= ~COM3B1;
  else if(pin == 3) TCCR3A &= ~COM3C1;
  else if(pin == 4) TCCR0A &= ~COM0B1;
  else if(pin == 5) TCCR3A &= ~COM3A1;
  else if(pin == 6) TCCR4A &= ~COM4A1;
  else if(pin == 7) TCCR4A &= ~COM4B1;
  else if(pin == 8) TCCR4A &= ~COM4C1;
  else if(pin == 9) TCCR2A &= ~COM2B1;
  else if(pin == 10) TCCR2A &= ~COM2A1;
  else if(pin == 11) TCCR1A &= ~COM1A1;
  else if(pin == 12) TCCR1A &= ~COM1B1;
  else if(pin == 13) TCCR0A &= ~COM0A1;
  else if(pin == 44) TCCR5A &= ~COM5C1;
  else if(pin == 45) TCCR5A &= ~COM5B1;
  else if(pin == 46) TCCR5A &= ~COM5A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   LilyPadUSB
 *   LilyPad Arduino USB
 *   MCU: atmega32u4
 */
#if defined(F_CPU) && (F_CPU+0) == 8000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 12 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && defined(USB_PID) && (USB_PID+0) == 0x9208
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 0) DDRD &= ~(1 << (2));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 1) DDRD &= ~(1 << (3));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 2) DDRD &= ~(1 << (1));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 3) DDRD &= ~(1 << (0));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRC |= (1 << (6));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (6));
    PORTC &= ~(1 << (6));
  } else if(pin == 5) DDRC &= ~(1 << (6));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 6) DDRD &= ~(1 << (7));
  else if(pin == 7 && mode == INPUT) DDRE |= (1 << (6));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (6));
    PORTE &= ~(1 << (6));
  } else if(pin == 7) DDRE &= ~(1 << (6));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 8) DDRB &= ~(1 << (4));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 9) DDRB &= ~(1 << (5));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 10) DDRB &= ~(1 << (6));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (7));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (7));
    PORTB &= ~(1 << (7));
  } else if(pin == 11) DDRB &= ~(1 << (7));
  else if(pin == 12 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 12) DDRD &= ~(1 << (6));
  else if(pin == 13 && mode == INPUT) DDRC |= (1 << (7));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (7));
    PORTC &= ~(1 << (7));
  } else if(pin == 13) DDRC &= ~(1 << (7));
  else if(pin == 14 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 14) DDRB &= ~(1 << (3));
  else if(pin == 15 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 15) DDRB &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 16) DDRB &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 17) DDRB &= ~(1 << (0));
  else if(pin == 18 && mode == INPUT) DDRF |= (1 << (7));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (7));
    PORTF &= ~(1 << (7));
  } else if(pin == 18) DDRF &= ~(1 << (7));
  else if(pin == 19 && mode == INPUT) DDRF |= (1 << (6));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (6));
    PORTF &= ~(1 << (6));
  } else if(pin == 19) DDRF &= ~(1 << (6));
  else if(pin == 20 && mode == INPUT) DDRF |= (1 << (5));
  else if(pin == 20 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (5));
    PORTF &= ~(1 << (5));
  } else if(pin == 20) DDRF &= ~(1 << (5));
  else if(pin == 21 && mode == INPUT) DDRF |= (1 << (4));
  else if(pin == 21 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (4));
    PORTF &= ~(1 << (4));
  } else if(pin == 21) DDRF &= ~(1 << (4));
  else if(pin == 22 && mode == INPUT) DDRF |= (1 << (1));
  else if(pin == 22 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (1));
    PORTF &= ~(1 << (1));
  } else if(pin == 22) DDRF &= ~(1 << (1));
  else if(pin == 23 && mode == INPUT) DDRF |= (1 << (0));
  else if(pin == 23 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (0));
    PORTF &= ~(1 << (0));
  } else if(pin == 23) DDRF &= ~(1 << (0));
  else if(pin == 24 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 24 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 24) DDRD &= ~(1 << (4));
  else if(pin == 25 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 25 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 25) DDRD &= ~(1 << (7));
  else if(pin == 26 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 26 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 26) DDRB &= ~(1 << (4));
  else if(pin == 27 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 27 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 27) DDRB &= ~(1 << (5));
  else if(pin == 28 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 28 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 28) DDRB &= ~(1 << (6));
  else if(pin == 29 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 29 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 29) DDRD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (2));
  else if(pin == 0 && !value) PORTD &= ~(1 << (2));
  else if(pin == 1 && value) PORTD  |= (1 << (3));
  else if(pin == 1 && !value) PORTD &= ~(1 << (3));
  else if(pin == 2 && value) PORTD  |= (1 << (1));
  else if(pin == 2 && !value) PORTD &= ~(1 << (1));
  else if(pin == 3 && value) PORTD  |= (1 << (0));
  else if(pin == 3 && !value) PORTD &= ~(1 << (0));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTC  |= (1 << (6));
  else if(pin == 5 && !value) PORTC &= ~(1 << (6));
  else if(pin == 6 && value) PORTD  |= (1 << (7));
  else if(pin == 6 && !value) PORTD &= ~(1 << (7));
  else if(pin == 7 && value) PORTE  |= (1 << (6));
  else if(pin == 7 && !value) PORTE &= ~(1 << (6));
  else if(pin == 8 && value) PORTB  |= (1 << (4));
  else if(pin == 8 && !value) PORTB &= ~(1 << (4));
  else if(pin == 9 && value) PORTB  |= (1 << (5));
  else if(pin == 9 && !value) PORTB &= ~(1 << (5));
  else if(pin == 10 && value) PORTB  |= (1 << (6));
  else if(pin == 10 && !value) PORTB &= ~(1 << (6));
  else if(pin == 11 && value) PORTB  |= (1 << (7));
  else if(pin == 11 && !value) PORTB &= ~(1 << (7));
  else if(pin == 12 && value) PORTD  |= (1 << (6));
  else if(pin == 12 && !value) PORTD &= ~(1 << (6));
  else if(pin == 13 && value) PORTC  |= (1 << (7));
  else if(pin == 13 && !value) PORTC &= ~(1 << (7));
  else if(pin == 14 && value) PORTB  |= (1 << (3));
  else if(pin == 14 && !value) PORTB &= ~(1 << (3));
  else if(pin == 15 && value) PORTB  |= (1 << (1));
  else if(pin == 15 && !value) PORTB &= ~(1 << (1));
  else if(pin == 16 && value) PORTB  |= (1 << (2));
  else if(pin == 16 && !value) PORTB &= ~(1 << (2));
  else if(pin == 17 && value) PORTB  |= (1 << (0));
  else if(pin == 17 && !value) PORTB &= ~(1 << (0));
  else if(pin == 18 && value) PORTF  |= (1 << (7));
  else if(pin == 18 && !value) PORTF &= ~(1 << (7));
  else if(pin == 19 && value) PORTF  |= (1 << (6));
  else if(pin == 19 && !value) PORTF &= ~(1 << (6));
  else if(pin == 20 && value) PORTF  |= (1 << (5));
  else if(pin == 20 && !value) PORTF &= ~(1 << (5));
  else if(pin == 21 && value) PORTF  |= (1 << (4));
  else if(pin == 21 && !value) PORTF &= ~(1 << (4));
  else if(pin == 22 && value) PORTF  |= (1 << (1));
  else if(pin == 22 && !value) PORTF &= ~(1 << (1));
  else if(pin == 23 && value) PORTF  |= (1 << (0));
  else if(pin == 23 && !value) PORTF &= ~(1 << (0));
  else if(pin == 24 && value) PORTD  |= (1 << (4));
  else if(pin == 24 && !value) PORTD &= ~(1 << (4));
  else if(pin == 25 && value) PORTD  |= (1 << (7));
  else if(pin == 25 && !value) PORTD &= ~(1 << (7));
  else if(pin == 26 && value) PORTB  |= (1 << (4));
  else if(pin == 26 && !value) PORTB &= ~(1 << (4));
  else if(pin == 27 && value) PORTB  |= (1 << (5));
  else if(pin == 27 && !value) PORTB &= ~(1 << (5));
  else if(pin == 28 && value) PORTB  |= (1 << (6));
  else if(pin == 28 && !value) PORTB &= ~(1 << (6));
  else if(pin == 29 && value) PORTD  |= (1 << (6));
  else if(pin == 29 && !value) PORTD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PINC & (1 << (6)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 7) return PINE & (1 << (6)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (7)) ? HIGH : LOW;
  else if(pin == 12) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 13) return PINC & (1 << (7)) ? HIGH : LOW;
  else if(pin == 14) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 15) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 18) return PINF & (1 << (7)) ? HIGH : LOW;
  else if(pin == 19) return PINF & (1 << (6)) ? HIGH : LOW;
  else if(pin == 20) return PINF & (1 << (5)) ? HIGH : LOW;
  else if(pin == 21) return PINF & (1 << (4)) ? HIGH : LOW;
  else if(pin == 22) return PINF & (1 << (1)) ? HIGH : LOW;
  else if(pin == 23) return PINF & (1 << (0)) ? HIGH : LOW;
  else if(pin == 24) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 25) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 26) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 27) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 28) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 29) return PIND & (1 << (6)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR0A &= ~COM0B1;
  else if(pin == 5) TCCR3A &= ~COM3A1;
  else if(pin == 6) TCCR4C &= ~COM4D1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR0A &= ~COM0A1;
  else if(pin == 13) TCCR4A &= ~COM4A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   micro
 *   Arduino Micro
 *   MCU: atmega32u4
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 12 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && defined(USB_PID) && (USB_PID+0) == 0x8037
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 0) DDRD &= ~(1 << (2));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 1) DDRD &= ~(1 << (3));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 2) DDRD &= ~(1 << (1));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 3) DDRD &= ~(1 << (0));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRC |= (1 << (6));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (6));
    PORTC &= ~(1 << (6));
  } else if(pin == 5) DDRC &= ~(1 << (6));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 6) DDRD &= ~(1 << (7));
  else if(pin == 7 && mode == INPUT) DDRE |= (1 << (6));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (6));
    PORTE &= ~(1 << (6));
  } else if(pin == 7) DDRE &= ~(1 << (6));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 8) DDRB &= ~(1 << (4));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 9) DDRB &= ~(1 << (5));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 10) DDRB &= ~(1 << (6));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (7));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (7));
    PORTB &= ~(1 << (7));
  } else if(pin == 11) DDRB &= ~(1 << (7));
  else if(pin == 12 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 12) DDRD &= ~(1 << (6));
  else if(pin == 13 && mode == INPUT) DDRC |= (1 << (7));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (7));
    PORTC &= ~(1 << (7));
  } else if(pin == 13) DDRC &= ~(1 << (7));
  else if(pin == 14 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 14) DDRB &= ~(1 << (3));
  else if(pin == 15 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 15) DDRB &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 16) DDRB &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 17) DDRB &= ~(1 << (0));
  else if(pin == 18 && mode == INPUT) DDRF |= (1 << (7));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (7));
    PORTF &= ~(1 << (7));
  } else if(pin == 18) DDRF &= ~(1 << (7));
  else if(pin == 19 && mode == INPUT) DDRF |= (1 << (6));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (6));
    PORTF &= ~(1 << (6));
  } else if(pin == 19) DDRF &= ~(1 << (6));
  else if(pin == 20 && mode == INPUT) DDRF |= (1 << (5));
  else if(pin == 20 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (5));
    PORTF &= ~(1 << (5));
  } else if(pin == 20) DDRF &= ~(1 << (5));
  else if(pin == 21 && mode == INPUT) DDRF |= (1 << (4));
  else if(pin == 21 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (4));
    PORTF &= ~(1 << (4));
  } else if(pin == 21) DDRF &= ~(1 << (4));
  else if(pin == 22 && mode == INPUT) DDRF |= (1 << (1));
  else if(pin == 22 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (1));
    PORTF &= ~(1 << (1));
  } else if(pin == 22) DDRF &= ~(1 << (1));
  else if(pin == 23 && mode == INPUT) DDRF |= (1 << (0));
  else if(pin == 23 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (0));
    PORTF &= ~(1 << (0));
  } else if(pin == 23) DDRF &= ~(1 << (0));
  else if(pin == 24 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 24 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 24) DDRD &= ~(1 << (4));
  else if(pin == 25 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 25 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 25) DDRD &= ~(1 << (7));
  else if(pin == 26 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 26 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 26) DDRB &= ~(1 << (4));
  else if(pin == 27 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 27 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 27) DDRB &= ~(1 << (5));
  else if(pin == 28 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 28 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 28) DDRB &= ~(1 << (6));
  else if(pin == 29 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 29 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 29) DDRD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (2));
  else if(pin == 0 && !value) PORTD &= ~(1 << (2));
  else if(pin == 1 && value) PORTD  |= (1 << (3));
  else if(pin == 1 && !value) PORTD &= ~(1 << (3));
  else if(pin == 2 && value) PORTD  |= (1 << (1));
  else if(pin == 2 && !value) PORTD &= ~(1 << (1));
  else if(pin == 3 && value) PORTD  |= (1 << (0));
  else if(pin == 3 && !value) PORTD &= ~(1 << (0));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTC  |= (1 << (6));
  else if(pin == 5 && !value) PORTC &= ~(1 << (6));
  else if(pin == 6 && value) PORTD  |= (1 << (7));
  else if(pin == 6 && !value) PORTD &= ~(1 << (7));
  else if(pin == 7 && value) PORTE  |= (1 << (6));
  else if(pin == 7 && !value) PORTE &= ~(1 << (6));
  else if(pin == 8 && value) PORTB  |= (1 << (4));
  else if(pin == 8 && !value) PORTB &= ~(1 << (4));
  else if(pin == 9 && value) PORTB  |= (1 << (5));
  else if(pin == 9 && !value) PORTB &= ~(1 << (5));
  else if(pin == 10 && value) PORTB  |= (1 << (6));
  else if(pin == 10 && !value) PORTB &= ~(1 << (6));
  else if(pin == 11 && value) PORTB  |= (1 << (7));
  else if(pin == 11 && !value) PORTB &= ~(1 << (7));
  else if(pin == 12 && value) PORTD  |= (1 << (6));
  else if(pin == 12 && !value) PORTD &= ~(1 << (6));
  else if(pin == 13 && value) PORTC  |= (1 << (7));
  else if(pin == 13 && !value) PORTC &= ~(1 << (7));
  else if(pin == 14 && value) PORTB  |= (1 << (3));
  else if(pin == 14 && !value) PORTB &= ~(1 << (3));
  else if(pin == 15 && value) PORTB  |= (1 << (1));
  else if(pin == 15 && !value) PORTB &= ~(1 << (1));
  else if(pin == 16 && value) PORTB  |= (1 << (2));
  else if(pin == 16 && !value) PORTB &= ~(1 << (2));
  else if(pin == 17 && value) PORTB  |= (1 << (0));
  else if(pin == 17 && !value) PORTB &= ~(1 << (0));
  else if(pin == 18 && value) PORTF  |= (1 << (7));
  else if(pin == 18 && !value) PORTF &= ~(1 << (7));
  else if(pin == 19 && value) PORTF  |= (1 << (6));
  else if(pin == 19 && !value) PORTF &= ~(1 << (6));
  else if(pin == 20 && value) PORTF  |= (1 << (5));
  else if(pin == 20 && !value) PORTF &= ~(1 << (5));
  else if(pin == 21 && value) PORTF  |= (1 << (4));
  else if(pin == 21 && !value) PORTF &= ~(1 << (4));
  else if(pin == 22 && value) PORTF  |= (1 << (1));
  else if(pin == 22 && !value) PORTF &= ~(1 << (1));
  else if(pin == 23 && value) PORTF  |= (1 << (0));
  else if(pin == 23 && !value) PORTF &= ~(1 << (0));
  else if(pin == 24 && value) PORTD  |= (1 << (4));
  else if(pin == 24 && !value) PORTD &= ~(1 << (4));
  else if(pin == 25 && value) PORTD  |= (1 << (7));
  else if(pin == 25 && !value) PORTD &= ~(1 << (7));
  else if(pin == 26 && value) PORTB  |= (1 << (4));
  else if(pin == 26 && !value) PORTB &= ~(1 << (4));
  else if(pin == 27 && value) PORTB  |= (1 << (5));
  else if(pin == 27 && !value) PORTB &= ~(1 << (5));
  else if(pin == 28 && value) PORTB  |= (1 << (6));
  else if(pin == 28 && !value) PORTB &= ~(1 << (6));
  else if(pin == 29 && value) PORTD  |= (1 << (6));
  else if(pin == 29 && !value) PORTD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PINC & (1 << (6)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 7) return PINE & (1 << (6)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (7)) ? HIGH : LOW;
  else if(pin == 12) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 13) return PINC & (1 << (7)) ? HIGH : LOW;
  else if(pin == 14) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 15) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 18) return PINF & (1 << (7)) ? HIGH : LOW;
  else if(pin == 19) return PINF & (1 << (6)) ? HIGH : LOW;
  else if(pin == 20) return PINF & (1 << (5)) ? HIGH : LOW;
  else if(pin == 21) return PINF & (1 << (4)) ? HIGH : LOW;
  else if(pin == 22) return PINF & (1 << (1)) ? HIGH : LOW;
  else if(pin == 23) return PINF & (1 << (0)) ? HIGH : LOW;
  else if(pin == 24) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 25) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 26) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 27) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 28) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 29) return PIND & (1 << (6)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR0A &= ~COM0B1;
  else if(pin == 5) TCCR3A &= ~COM3A1;
  else if(pin == 6) TCCR4C &= ~COM4D1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR0A &= ~COM0A1;
  else if(pin == 13) TCCR4A &= ~COM4A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
/* Arduino board:
 *   leonardo
 *   Arduino Leonardo
 *   MCU: atmega32u4
 */
#if defined(F_CPU) && (F_CPU+0) == 16000000L && defined(NUM_ANALOG_INPUTS) && (NUM_ANALOG_INPUTS+0) == 12 && defined(SIGNATURE_1) && (SIGNATURE_1+0) == 0x95 && defined(USB_PID) && (USB_PID+0) == 0x8036
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  else if(pin == 0 && mode == INPUT) DDRD |= (1 << (2));
  else if(pin == 0 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (2));
    PORTD &= ~(1 << (2));
  } else if(pin == 0) DDRD &= ~(1 << (2));
  else if(pin == 1 && mode == INPUT) DDRD |= (1 << (3));
  else if(pin == 1 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (3));
    PORTD &= ~(1 << (3));
  } else if(pin == 1) DDRD &= ~(1 << (3));
  else if(pin == 2 && mode == INPUT) DDRD |= (1 << (1));
  else if(pin == 2 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (1));
    PORTD &= ~(1 << (1));
  } else if(pin == 2) DDRD &= ~(1 << (1));
  else if(pin == 3 && mode == INPUT) DDRD |= (1 << (0));
  else if(pin == 3 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (0));
    PORTD &= ~(1 << (0));
  } else if(pin == 3) DDRD &= ~(1 << (0));
  else if(pin == 4 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 4 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 4) DDRD &= ~(1 << (4));
  else if(pin == 5 && mode == INPUT) DDRC |= (1 << (6));
  else if(pin == 5 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (6));
    PORTC &= ~(1 << (6));
  } else if(pin == 5) DDRC &= ~(1 << (6));
  else if(pin == 6 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 6 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 6) DDRD &= ~(1 << (7));
  else if(pin == 7 && mode == INPUT) DDRE |= (1 << (6));
  else if(pin == 7 && mode == INPUT_PULLUP) {
    DDRE |= (1 << (6));
    PORTE &= ~(1 << (6));
  } else if(pin == 7) DDRE &= ~(1 << (6));
  else if(pin == 8 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 8 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 8) DDRB &= ~(1 << (4));
  else if(pin == 9 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 9 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 9) DDRB &= ~(1 << (5));
  else if(pin == 10 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 10 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 10) DDRB &= ~(1 << (6));
  else if(pin == 11 && mode == INPUT) DDRB |= (1 << (7));
  else if(pin == 11 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (7));
    PORTB &= ~(1 << (7));
  } else if(pin == 11) DDRB &= ~(1 << (7));
  else if(pin == 12 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 12 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 12) DDRD &= ~(1 << (6));
  else if(pin == 13 && mode == INPUT) DDRC |= (1 << (7));
  else if(pin == 13 && mode == INPUT_PULLUP) {
    DDRC |= (1 << (7));
    PORTC &= ~(1 << (7));
  } else if(pin == 13) DDRC &= ~(1 << (7));
  else if(pin == 14 && mode == INPUT) DDRB |= (1 << (3));
  else if(pin == 14 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (3));
    PORTB &= ~(1 << (3));
  } else if(pin == 14) DDRB &= ~(1 << (3));
  else if(pin == 15 && mode == INPUT) DDRB |= (1 << (1));
  else if(pin == 15 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (1));
    PORTB &= ~(1 << (1));
  } else if(pin == 15) DDRB &= ~(1 << (1));
  else if(pin == 16 && mode == INPUT) DDRB |= (1 << (2));
  else if(pin == 16 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (2));
    PORTB &= ~(1 << (2));
  } else if(pin == 16) DDRB &= ~(1 << (2));
  else if(pin == 17 && mode == INPUT) DDRB |= (1 << (0));
  else if(pin == 17 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (0));
    PORTB &= ~(1 << (0));
  } else if(pin == 17) DDRB &= ~(1 << (0));
  else if(pin == 18 && mode == INPUT) DDRF |= (1 << (7));
  else if(pin == 18 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (7));
    PORTF &= ~(1 << (7));
  } else if(pin == 18) DDRF &= ~(1 << (7));
  else if(pin == 19 && mode == INPUT) DDRF |= (1 << (6));
  else if(pin == 19 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (6));
    PORTF &= ~(1 << (6));
  } else if(pin == 19) DDRF &= ~(1 << (6));
  else if(pin == 20 && mode == INPUT) DDRF |= (1 << (5));
  else if(pin == 20 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (5));
    PORTF &= ~(1 << (5));
  } else if(pin == 20) DDRF &= ~(1 << (5));
  else if(pin == 21 && mode == INPUT) DDRF |= (1 << (4));
  else if(pin == 21 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (4));
    PORTF &= ~(1 << (4));
  } else if(pin == 21) DDRF &= ~(1 << (4));
  else if(pin == 22 && mode == INPUT) DDRF |= (1 << (1));
  else if(pin == 22 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (1));
    PORTF &= ~(1 << (1));
  } else if(pin == 22) DDRF &= ~(1 << (1));
  else if(pin == 23 && mode == INPUT) DDRF |= (1 << (0));
  else if(pin == 23 && mode == INPUT_PULLUP) {
    DDRF |= (1 << (0));
    PORTF &= ~(1 << (0));
  } else if(pin == 23) DDRF &= ~(1 << (0));
  else if(pin == 24 && mode == INPUT) DDRD |= (1 << (4));
  else if(pin == 24 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (4));
    PORTD &= ~(1 << (4));
  } else if(pin == 24) DDRD &= ~(1 << (4));
  else if(pin == 25 && mode == INPUT) DDRD |= (1 << (7));
  else if(pin == 25 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (7));
    PORTD &= ~(1 << (7));
  } else if(pin == 25) DDRD &= ~(1 << (7));
  else if(pin == 26 && mode == INPUT) DDRB |= (1 << (4));
  else if(pin == 26 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (4));
    PORTB &= ~(1 << (4));
  } else if(pin == 26) DDRB &= ~(1 << (4));
  else if(pin == 27 && mode == INPUT) DDRB |= (1 << (5));
  else if(pin == 27 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (5));
    PORTB &= ~(1 << (5));
  } else if(pin == 27) DDRB &= ~(1 << (5));
  else if(pin == 28 && mode == INPUT) DDRB |= (1 << (6));
  else if(pin == 28 && mode == INPUT_PULLUP) {
    DDRB |= (1 << (6));
    PORTB &= ~(1 << (6));
  } else if(pin == 28) DDRB &= ~(1 << (6));
  else if(pin == 29 && mode == INPUT) DDRD |= (1 << (6));
  else if(pin == 29 && mode == INPUT_PULLUP) {
    DDRD |= (1 << (6));
    PORTD &= ~(1 << (6));
  } else if(pin == 29) DDRD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else if(pin == 0 && value) PORTD  |= (1 << (2));
  else if(pin == 0 && !value) PORTD &= ~(1 << (2));
  else if(pin == 1 && value) PORTD  |= (1 << (3));
  else if(pin == 1 && !value) PORTD &= ~(1 << (3));
  else if(pin == 2 && value) PORTD  |= (1 << (1));
  else if(pin == 2 && !value) PORTD &= ~(1 << (1));
  else if(pin == 3 && value) PORTD  |= (1 << (0));
  else if(pin == 3 && !value) PORTD &= ~(1 << (0));
  else if(pin == 4 && value) PORTD  |= (1 << (4));
  else if(pin == 4 && !value) PORTD &= ~(1 << (4));
  else if(pin == 5 && value) PORTC  |= (1 << (6));
  else if(pin == 5 && !value) PORTC &= ~(1 << (6));
  else if(pin == 6 && value) PORTD  |= (1 << (7));
  else if(pin == 6 && !value) PORTD &= ~(1 << (7));
  else if(pin == 7 && value) PORTE  |= (1 << (6));
  else if(pin == 7 && !value) PORTE &= ~(1 << (6));
  else if(pin == 8 && value) PORTB  |= (1 << (4));
  else if(pin == 8 && !value) PORTB &= ~(1 << (4));
  else if(pin == 9 && value) PORTB  |= (1 << (5));
  else if(pin == 9 && !value) PORTB &= ~(1 << (5));
  else if(pin == 10 && value) PORTB  |= (1 << (6));
  else if(pin == 10 && !value) PORTB &= ~(1 << (6));
  else if(pin == 11 && value) PORTB  |= (1 << (7));
  else if(pin == 11 && !value) PORTB &= ~(1 << (7));
  else if(pin == 12 && value) PORTD  |= (1 << (6));
  else if(pin == 12 && !value) PORTD &= ~(1 << (6));
  else if(pin == 13 && value) PORTC  |= (1 << (7));
  else if(pin == 13 && !value) PORTC &= ~(1 << (7));
  else if(pin == 14 && value) PORTB  |= (1 << (3));
  else if(pin == 14 && !value) PORTB &= ~(1 << (3));
  else if(pin == 15 && value) PORTB  |= (1 << (1));
  else if(pin == 15 && !value) PORTB &= ~(1 << (1));
  else if(pin == 16 && value) PORTB  |= (1 << (2));
  else if(pin == 16 && !value) PORTB &= ~(1 << (2));
  else if(pin == 17 && value) PORTB  |= (1 << (0));
  else if(pin == 17 && !value) PORTB &= ~(1 << (0));
  else if(pin == 18 && value) PORTF  |= (1 << (7));
  else if(pin == 18 && !value) PORTF &= ~(1 << (7));
  else if(pin == 19 && value) PORTF  |= (1 << (6));
  else if(pin == 19 && !value) PORTF &= ~(1 << (6));
  else if(pin == 20 && value) PORTF  |= (1 << (5));
  else if(pin == 20 && !value) PORTF &= ~(1 << (5));
  else if(pin == 21 && value) PORTF  |= (1 << (4));
  else if(pin == 21 && !value) PORTF &= ~(1 << (4));
  else if(pin == 22 && value) PORTF  |= (1 << (1));
  else if(pin == 22 && !value) PORTF &= ~(1 << (1));
  else if(pin == 23 && value) PORTF  |= (1 << (0));
  else if(pin == 23 && !value) PORTF &= ~(1 << (0));
  else if(pin == 24 && value) PORTD  |= (1 << (4));
  else if(pin == 24 && !value) PORTD &= ~(1 << (4));
  else if(pin == 25 && value) PORTD  |= (1 << (7));
  else if(pin == 25 && !value) PORTD &= ~(1 << (7));
  else if(pin == 26 && value) PORTB  |= (1 << (4));
  else if(pin == 26 && !value) PORTB &= ~(1 << (4));
  else if(pin == 27 && value) PORTB  |= (1 << (5));
  else if(pin == 27 && !value) PORTB &= ~(1 << (5));
  else if(pin == 28 && value) PORTB  |= (1 << (6));
  else if(pin == 28 && !value) PORTB &= ~(1 << (6));
  else if(pin == 29 && value) PORTD  |= (1 << (6));
  else if(pin == 29 && !value) PORTD &= ~(1 << (6));

}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else if(pin == 0) return PIND & (1 << (2)) ? HIGH : LOW;
  else if(pin == 1) return PIND & (1 << (3)) ? HIGH : LOW;
  else if(pin == 2) return PIND & (1 << (1)) ? HIGH : LOW;
  else if(pin == 3) return PIND & (1 << (0)) ? HIGH : LOW;
  else if(pin == 4) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 5) return PINC & (1 << (6)) ? HIGH : LOW;
  else if(pin == 6) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 7) return PINE & (1 << (6)) ? HIGH : LOW;
  else if(pin == 8) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 9) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 10) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 11) return PINB & (1 << (7)) ? HIGH : LOW;
  else if(pin == 12) return PIND & (1 << (6)) ? HIGH : LOW;
  else if(pin == 13) return PINC & (1 << (7)) ? HIGH : LOW;
  else if(pin == 14) return PINB & (1 << (3)) ? HIGH : LOW;
  else if(pin == 15) return PINB & (1 << (1)) ? HIGH : LOW;
  else if(pin == 16) return PINB & (1 << (2)) ? HIGH : LOW;
  else if(pin == 17) return PINB & (1 << (0)) ? HIGH : LOW;
  else if(pin == 18) return PINF & (1 << (7)) ? HIGH : LOW;
  else if(pin == 19) return PINF & (1 << (6)) ? HIGH : LOW;
  else if(pin == 20) return PINF & (1 << (5)) ? HIGH : LOW;
  else if(pin == 21) return PINF & (1 << (4)) ? HIGH : LOW;
  else if(pin == 22) return PINF & (1 << (1)) ? HIGH : LOW;
  else if(pin == 23) return PINF & (1 << (0)) ? HIGH : LOW;
  else if(pin == 24) return PIND & (1 << (4)) ? HIGH : LOW;
  else if(pin == 25) return PIND & (1 << (7)) ? HIGH : LOW;
  else if(pin == 26) return PINB & (1 << (4)) ? HIGH : LOW;
  else if(pin == 27) return PINB & (1 << (5)) ? HIGH : LOW;
  else if(pin == 28) return PINB & (1 << (6)) ? HIGH : LOW;
  else if(pin == 29) return PIND & (1 << (6)) ? HIGH : LOW;

  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
  else if(pin == 3) TCCR0A &= ~COM0B1;
  else if(pin == 5) TCCR3A &= ~COM3A1;
  else if(pin == 6) TCCR4C &= ~COM4D1;
  else if(pin == 9) TCCR1A &= ~COM1A1;
  else if(pin == 10) TCCR1A &= ~COM1B1;
  else if(pin == 11) TCCR0A &= ~COM0A1;
  else if(pin == 13) TCCR4A &= ~COM4A1;

}

__attribute__((always_inline))
static inline void pinModeSafe(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(mode == INPUT) { // Don't let input pins stay in PWM mode
      noAnalogWrite(pin);
    }
    pinModeFast(pin, mode);
  }
}

__attribute__((always_inline))
static inline void digitalWriteSafe(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      noAnalogWrite(pin);
      digitalWriteFast(pin, value);
    }
  }
}

__attribute__((always_inline))
static inline int digitalReadSafe(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      return digitalReadFast(pin);
    }
  }
}


#endif
#ifndef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics couldn't match this board configuration. No fast I/O is available. The header may be out of date."
#endif
#undef _DIGITALIO_MATCHED_BOARD

#ifndef DIGITALIO_MANUAL
#ifdef DIGITALIO_NO_INTERRUPTS
#define digitalWrite digitalWriteFast
#define digitalRead digitalReadFast
#define pinMode pinModeFast
#else
#define digitalWrite digitalWriteSafe
#define digitalRead digitalReadSafe
#define pinMode pinModeSafe
#endif

#endif

#endif
#endif

