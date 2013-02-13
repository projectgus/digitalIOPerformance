#!/usr/bin/env python

# TODO: command line options
ARDUINO_PATH="/usr/share/arduino"

import os, re, subprocess

RE_BOARDENTRY = re.compile(r"^([a-z][^=]+)=(.+)$", re.MULTILINE)

boards = open(os.path.join(ARDUINO_PATH, "hardware/arduino/boards.txt"))
models = dict()

for full_key,value in re.findall(RE_BOARDENTRY, boards.read()):
    model = full_key.split(".")[0]
    key = ".".join(full_key.split(".")[1:])
    if not model in models:
        models[model] = { "id" : model }
    models[model][key] = value

models = list(models.values())

mcus = set( m["build.mcu"] for m in models )
variants = set( m["build.variant"] for m in models )

def run_preprocessor(model, additional_args=[]):
    """
    Run the preprocessor and return the contents of the processed file as a string
    """
    source_path = model["pin_path"]
    args = ["avr-gcc", "-DARDUINO_MAIN", "-E",
            "-mmcu=%s" % model["build.mcu"],
            "-DF_CPU=%s" % model["build.f_cpu"] ]
    if "build.vid" in model:
        args += [ "-DUSB_VID=%s" % model["build.vid" ] ]
    if "build.pid" in model:
        args += [ "-DUSB_PID=%s" % model["build.pid" ] ]
    proc = subprocess.Popen(args + additional_args + [ source_path ],
                            stdout=subprocess.PIPE)
    proc.wait()
    if proc.returncode != 0:
        raise Error("Failed to run preprocessor")
    return proc.stdout.read()

def add_variant_macros(model):
    model["pin_path"] = os.path.join(ARDUINO_PATH, "hardware/arduino/variants/%s/pins_arduino.h" % (model["build.variant"]))
    macros = run_preprocessor(model, ["-dM"])
    macros = [ re.sub(r"^#define ", "", macro) for macro in macros.split("\n") ]
    macros = [ tuple(macro.split(" ", 1)) for macro in macros if " " in macro ]
    model["macros"] = dict(macros)

for model in models:
    add_variant_macros(model)

# Trim the macros we care about to remove any with possible preprocessor ambiguity
# (spaces, parentheses)
ambiguous_macros = set()
for model in models:
    for key,value in model["macros"].items():
        if " " in value or "(" in value or ")" in value:
            ambiguous_macros.add(key)
for model in models:
    for ambiguous in ambiguous_macros:
        if ambiguous in model["macros"]:
            del model["macros"][ambiguous]

# Find unique identifiable macro values that distinguish each model from the others
identifying_keys = set()
for model in models:
    duplicates = list( dup for dup in models if dup != model )
    for dup in duplicates:
        # skip anything that's already unique as per existing keys
        identified = False
        for key in identifying_keys:
            identified = identified or model["macros"].get(key,"") != dup["macros"].get(key, "")
        if identified:
            continue

        # find something unique about this duplicate
        uniques = set(model["macros"].items()) ^ set(dup["macros"].items())
        uniques = [ key for key,value in uniques ]
        # find the least selective key in the uniques
        def selectiveness(key):
            return len([d for d in duplicates if ( d["macros"].get(key, "") == model["macros"].get(key, ""))])
        uniques = sorted(uniques, key=selectiveness)
        if len(uniques) > 0:
            identifying_keys.add(uniques[0])


# Apply the key as a string to each model
for model in models:
    model["key"] = str([ model["macros"].get(key,"") for key in identifying_keys ])

# Merge together any models with identical keys (by name & id)
unique_models = []
for model in models:
    found = False
    for unique in unique_models:
        if unique["key"] == model["key"]:
            if model["build.variant"] != unique["build.variant"]:
                raise RuntimeError("Ambiguous models %s / %s have matching variant! Can't distinguish reliably between them." % (model["id"], unique["id"]))
            unique["id"] += " | " + model["id"]
            unique["name"] += " | " + model["name"]
            found = True
    if not found:
        unique_models += [ model ]

# Run the preprocessor over the pin header files to pull out port names and
# bitmasks
for model in models:
    output = run_preprocessor(model)
    digital_pin_to_port = re.search(r"digital_pin_to_port_PGM.+?\{(.+?)}",
                                       output, re.MULTILINE|re.DOTALL).group(1)
    digital_pin_to_bit_mask = re.search(r"digital_pin_to_bit_mask_PGM.+?\{(.+?)\}",
                                           output, re.MULTILINE|re.DOTALL).group(1)
    model["ports"] = [ e.strip() for e in digital_pin_to_port.split(",")
                       if len(e.strip()) > 0 ]
    # strip P prefix, ie PD becomes D
    model["ports"] = [ e[1] if e[0] == "P" else None for e in model["ports"] ]
    model["bitmasks"] = [ e.strip() for e in digital_pin_to_bit_mask.split(",")
                          if len(e.strip()) > 0 ]
    # do some sanity checks
    if len(model["ports"]) != len(model["bitmasks"]):
        raise RuntimeError("Number of ports in %s doesn't match bitmask count - %d vs %d" % (model["id"], len(model["ports"]), len(model["bitmasks"])))
    if len(model["ports"]) != int(model["macros"]["NUM_DIGITAL_PINS"]):
        raise RuntimeError("Number of ports in %s doesn't match number of digital pins reported in header" % (model["id"], len(model["ports"]), model["macros"]["NUM_DIGITAL_PINS"]))
    #print model["id"],model["ports"],model["bitmasks"]


PRELUDE = """
/*
 * Autogenerated header
 */

"""

MODEL_TEMPLATE = """
/* Arduino model:
 *   %(id)s
 *   %(name)s
 *   MCU: %(build.mcu)s
 */
#if %(ifdef_clause)s
#if %(ifvalue_clause)s

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

#endif
#endif

"""

DIGITALWRITE_TEMPLATE = """
  else if(pin == %(number)s && value) PORT%(port)s |= %(bitmask)s;
  else if(pin == %(number)s && !value) PORT%(port)s &= ~%(bitmask)s;
""".strip("\n")

PINMODE_TEMPLATE = """
  else if(pin == %(number)s && mode) DDR%(port)s |= %(bitmask)s;
  else if(pin == %(number)s && !mode) DDR%(port)s &= ~%(bitmask)s;
""".strip("\n")

DIGITALREAD_TEMPLATE = """
  else if(pin == %(number)s) return PIN%(port)s & %(bitmask)s ? HIGH : LOW;
""".strip("\n")

with open("digitalIOPerformance.h", "w") as output:
    output.write(PRELUDE)
    for model in models:
        # Work out the macro conditional
        ifdef_clause = ["1"]
        ifvalue_clause = ["1"]
        for key in identifying_keys:
            # all the +0 nonsense here is to detect empty macros
            # (Arduino-mk inserts them), same for separating out ==
            # equality tests until we know the macro has a non-empty value
            if key in model["macros"]:
                ifdef_clause.append("defined(%s) && (%s+0)" % (key, key))
                ifvalue_clause.append("(%s == %s)" % (key, model["macros"][key]))
            else:
                ifdef_clause.append("(!defined(%s) || !(%s+0))" % (key,key))
        ifdef_clause = " && ".join(ifdef_clause)
        ifvalue_clause = " && ".join(ifvalue_clause)

        # Build up the macro conditionals
        digitalwrite_clause = ""
        digitalread_clause = ""
        pinmode_clause = ""
        for number,port,bitmask in zip(range(len(model["ports"])),
                                  model["ports"],
                                  model["bitmasks"]):
            digitalwrite_clause += DIGITALWRITE_TEMPLATE % locals() + "\n"
            digitalread_clause += DIGITALREAD_TEMPLATE % locals() + "\n"
            pinmode_clause += PINMODE_TEMPLATE % locals() + "\n"

        output.write(MODEL_TEMPLATE % dict(locals(), **model))
