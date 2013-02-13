/* Arduino board:
 *   %(id)s
 *   %(name)s
 *   MCU: %(build.mcu)s
 */
#if %(ifdef_clause)s
#ifdef _DIGITALIO_MATCHED_BOARD
#error "This header's Arduino configuration heuristics have matched multiple boards. The header may be out of date."
#endif
#define _DIGITALIO_MATCHED_BOARD

__attribute__((always_inline))
static inline void pinModeFast(uint8_t pin, uint8_t mode) {
  if(!__builtin_constant_p(pin)) {
    pinMode(pin, mode);
  }
%(pinmode_clause)s
}

__attribute__((always_inline))
static inline void digitalWriteFast(uint8_t pin, uint8_t value) {
  if(!__builtin_constant_p(pin)) {
    digitalWrite(pin, value);
  }
%(digitalwrite_clause)s
}

__attribute__((always_inline))
static inline int digitalReadFast(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return digitalRead(pin);
  }
%(digitalread_clause)s
  return LOW;
}

__attribute__((always_inline))
static inline void noAnalogWrite(uint8_t pin) {
  if(!__builtin_constant_p(pin)) {
    return; // noAnalogWrite is taken care of by digitalWrite() for variables
  }
%(timer_clause)s
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
