# digitalIOPerformance.h

digitalIOPerformance.h is a single-file Arduino library (header file)
that adds higher-performance digital I/O functions to Arduino
programs.

# Quick Start

* Copy the "digitalIOPerformance" directory to your [Arduino libraries folder](http://arduino.cc/en/Guide/Libraries).

* Add "#include "digitalIOPerformance.h" near the top of your sketch.

* Done! When you recompile, performance of digitalRead/digitalWrite &
  pinMode should be substantially faster in most cases. However,
  functionality should be otherwise identical to the original Arduino
  functions.

* Your sketch's compiled size may also go down (depending on how much
  digital I/O you do.)

## Option: Even better performance

If your Arduino sketch doesn't use interrupts, or you don't care about
interrupt safety for digital I/O, then you can add a second line above
the first to get even faster digital I/O:

    #define DIGITALIO_NO_INTERRUPTS
    #include "digitalIOPerformance.h

Performance of digitalRead/digitalWrite & pinMode will be faster still,
but interrupts won't be disabled while reading/writing and you'll have to call
noAnalogWrite(pin) if you're intermixing analogWrite and digitalWrite
on the same pin (see below.)

## Option: Disable automatic performance boost

If you don't want the library to automatically replace your
digitalRead/digitalWrite/pinMode function calls, you can do that as
well:

    #define DIGITALIO_MANUAL
    #include "digitalIOPerformance.h

You can still use the functions in the library if you call them by
their original names (given below.)

# Functions Defined

These functions are defined by the library:

## digitalWriteFast / digitalReadFast / pinModeFast

These versions of digitalWrite/digitalRead & pinMode will compile down
to a single port register instruction if the pin number is known at
compile time. If the pin number is a variable then they fall through
to the slower Arduino version if the pin number is a variable.

If you define "DIGITALIO_NO_INTERRUPTS" before you include the
library, these functions automatically replace the built-in
digitalWrite, digitalRead & pinMode.

## noAnalogWrite

Using digitalWriteFast() will not automatically turn off a
previous analogWrite() to that port, unlike Arduino's digitalWrite().

If you are mixing analogWrite() and digitalWriteFast() on a port, call
this function after immediately before calling digitalWriteFast(), if
you had previously called analogWrite().

## digitalWriteSafe / digitalReadSafe / pinModeSafe

These versions of digitalWrite/digitalRead & pinMode run faster
than the built-in Arduino versions, if the pin number is known at
compile time.

They are also just as safe as the built-in Arduino versions - they're
interrupt safe, and they disable any previous analogWrite() calls.

When you include the library, these functions automatically replace
the built-in digitalWrite, digitalRead & pinMode functions. If you
don't want this to happen, define DIGITALIO_MANUAL before including
(as shown above.)

The downside to using these is they take up marginally more room in
the compiled sketch. The overall sketch can still be substantially
smaller if it no longer uses any of the the built-in
analogRead/digitalRead/pinMode functions. However it can get larger if
you have a lot of function calls, or are still using libraries that
refer to the built-in functions.


# Status

New, untested, hacky, work in progress. :)

Please raise an issue if this doesn't work with your Arduino install,
or doesn't seem to inline properly (ie massively bloated code size
instead of shrinking code size!)

Minimal testing done with Windows Arduino 1.0.3 (gcc 4.3) and my
Ubuntu Arduino 1.0.3 (gcc 4.7.) Should work with any Arduino version
above 1.0 (I think.)

# Known Shortcomings

* No ARM support, definitely won't work on the Arduino Due.

# Internal Workings

digitalIOPerformance.h is code generated automatically from an
existing Arduino installation by the script generateDigitalIOHeader.py.

You shouldn't need to run the code generation script unless you have a
newer/different Arduino version than the one it was last run against.

However, having code generation means it should be simple to update
against future new boards like the Leonardo (assuming the file
formats don't change much.)

# Thanks

Big thanks to the authors of digitalWriteFast - Paul Stoffregen, Bill
Westfield, an John Raines. I wrote this instead of updating
digitalWriteFast.h to support the Leonardo (code generation was more
appealing than handwritten bit fiddles!)

Also thanks to Alastair D'Silva who told me a while ago about the
trick of preprocessing pins_arduino.h to extract Arduino pin
information, he uses this in the performance-oriented AVR library
[MHVLib](http://www.makehackvoid.com/project/MHVLib).
