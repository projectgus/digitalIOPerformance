# digitalIOPerformance.h

digitalIOPerformance.h is a header file that adds higher-performance
digital I/O functions to Arduino programs:

## digitalWriteFast() / digitalReadFast / pinModeFast

Versions of digitalWrite()/digitalRead/pinMode that compile down to a
single port register instruction if the pin number is known at compile
time (falls through to the slower Arduino version if the pin number is a variable.)

## turnOffPWM()
Using digitalWriteFast() will not automatically turn off a
previous analogWrite() to that port, unlike Arduino's digitalWrite().

If you are mixing analogWrite() and digitalWriteFast() on a port, call
this function before calling digitalWriteFast().

# Status

New, untested, hacky, work in progress. :)

Please raise an issue if this doesn't work with your Arduino install,
or doesn't inline properly (ie bloated code size instead of shrinking
code size!)

Minimal testing done with Windows Arduino 1.0.3 (gcc 4.3) and my
Ubuntu Arduino 1.0.3 (gcc 4.7.) Should work with any Arduino version
above 1.0 (I think.)

# Known Shortcomings

* Not yet interrupt-safe, you'll need to disable/restore interrupts
  yourself if you need interrupt safety.

* If you're mixing analogWrite() and digitalWriteFast() on a single
  pin then you'll need turnOffPWM() before calling digitalWriteFast(),
  as documented above.

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
