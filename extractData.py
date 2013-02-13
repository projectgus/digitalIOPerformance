#!/usr/bin/env python

# TODO: command line options
ARDUINO_PATH="/usr/share/arduino"

import os, re, subprocess

def main():
    boards = extract_boards()
    for board in boards:
        add_variant_macros(board)
    find_unambiguous_macros(boards)
    identifying_keys = find_unique_macro_keys(boards)
    boards = merge_matching_boards(boards, identifying_keys)
    for board in boards:
        extract_portnames_pins(board)
    with open("digitalIOPerformance.h", "w") as output:
        generate_header_file(boards, identifying_keys, output)

def extract_boards():
    """ Parse the Arduino boards file and return a list of all the boards,
    with each board as a dictionary containing all their board-specific
    key-value pairs
    """
    with open(os.path.join(ARDUINO_PATH, "hardware/arduino/boards.txt")) as board_contents:
        boards = dict()

        RE_BOARDENTRY = re.compile(r"^([A-Za-z][^=]+)=(.+)$", re.MULTILINE)

        for full_key,value in re.findall(RE_BOARDENTRY, board_contents.read()):
            board = full_key.split(".")[0]
            key = ".".join(full_key.split(".")[1:])
            if not board in boards:
                boards[board] = { "id" : board }
            boards[board][key] = value

    return list(boards.values())

def run_preprocessor(board, additional_args=[]):
    """
    Run the C preprocessor over a particular board, and return
    the contents of the processed file as a string
    """
    source_path = board["pin_path"]
    args = ["avr-gcc", "-DARDUINO_MAIN", "-E",
            "-mmcu=%s" % board["build.mcu"],
            "-DF_CPU=%s" % board["build.f_cpu"] ]
    if "build.vid" in board:
        args += [ "-DUSB_VID=%s" % board["build.vid" ] ]
    if "build.pid" in board:
        args += [ "-DUSB_PID=%s" % board["build.pid" ] ]
    proc = subprocess.Popen(args + additional_args + [ source_path ],
                            stdout=subprocess.PIPE)
    proc.wait()
    if proc.returncode != 0:
        raise Error("Failed to run preprocessor")
    return proc.stdout.read()

def add_variant_macros(board):
    """
    Run the pin_arduinos.h header for this board through the preprocessor,
    extract all defined macros and attach them to the board dict under key
    'macros'
    """
    board["pin_path"] = os.path.join(ARDUINO_PATH, "hardware/arduino/variants/%s/pins_arduino.h" % (board["build.variant"]))
    macros = run_preprocessor(board, ["-dM"])
    macros = [ re.sub(r"^#define ", "", macro) for macro in macros.split("\n") ]
    macros = [ tuple(macro.split(" ", 1)) for macro in macros if " " in macro ]
    board["macros"] = dict(macros)

def find_unambiguous_macros(boards):
    """
    Trim the macros defined against any of the boards, to leave only those with
    only numeric values (anything else is too tricky, especially with
    token representation ambiguities.)

    Modifies the board dict in-place.
    """
    ambiguous_macros = set()
    # build list of ambiguous macros
    for board in boards:
        for key,value in board["macros"].items():
            if not re.match(r"^-?0?x?[\dA-Fa-f]+L?$", value.strip()):
                ambiguous_macros.add(key)
    # trim the ambiguous macros from any of the boards
    for board in boards:
        for ambiguous in ambiguous_macros:
            if ambiguous in board["macros"]:
                del board["macros"][ambiguous]


def find_unique_macro_keys(boards):
    """
    Go through each board and find a small subset of unique macro
    values that distinguish each board from the others.

    This allows us to generate a header file that can automatically
    determine which board profile it's being compiled against, at compile
    time.

    Returns a set of macro names.
    """
    identifying_keys = set()
    for board in boards:
        duplicates = list( dup for dup in boards if dup != board )
        for dup in duplicates:
            # skip anything that's already unique as per existing keys
            identified = False
            for key in identifying_keys:
                identified = identified or board["macros"].get(key,"") != dup["macros"].get(key, "")
            if identified:
                continue

            # find something unique about this duplicate
            uniques = set(board["macros"].items()) ^ set(dup["macros"].items())
            uniques = [ key for key,value in uniques ]
            # find the least selective key in the uniques
            def selectiveness(key):
                return len([d for d in duplicates if ( d["macros"].get(key, "") == board["macros"].get(key, ""))])
            uniques = sorted(uniques, key=selectiveness)
            if len(uniques) > 0:
                identifying_keys.add(uniques[0])

    return identifying_keys

def merge_matching_boards(boards, identifying_keys):
    """
    Go through each board and merge together any that we can't unambiguously
    identify by using the macros defined in identifying_keys.

    We assume any matching boards will have matching 'variants' defined, otherwise
    we throw an error.

    Returns a new list of boards, with an ambiguously defined ones merged together.
    """
    def key(board):
        return str([ board["macros"].get(key,"") for key in identifying_keys ])

    # Merge together any boards with identical keys (making composite names & ids)
    unique_boards = []
    for board in boards:
        print key(board)
        found = False
        for unique in unique_boards:
            if key(unique) == key(board):
                if board["build.variant"] != unique["build.variant"]:
                    raise RuntimeError("Ambiguous boards %s / %s have matching variant! Can't distinguish reliably between them." % (board["id"], unique["id"]))
                unique["id"] += " | " + board["id"]
                unique["name"] += " | " + board["name"]
                found = True
        if not found:
            unique_boards += [ board ]
    return unique_boards


def extract_portnames_pins(board):
    """
    Run the preprocessor over this boards' pin header file to pull
    out port names and bitmasks.
    """
    output = run_preprocessor(board)
    digital_pin_to_port = re.search(r"digital_pin_to_port_PGM.+?\{(.+?)}",
                                       output, re.MULTILINE|re.DOTALL).group(1)
    digital_pin_to_bit_mask = re.search(r"digital_pin_to_bit_mask_PGM.+?\{(.+?)\}",
                                           output, re.MULTILINE|re.DOTALL).group(1)
    board["ports"] = [ e.strip() for e in digital_pin_to_port.split(",")
                       if len(e.strip()) > 0 ]
    # strip P prefix, ie PD becomes D
    board["ports"] = [ e[1] if e[0] == "P" else None for e in board["ports"] ]
    board["bitmasks"] = [ e.strip() for e in digital_pin_to_bit_mask.split(",")
                          if len(e.strip()) > 0 ]

    # do some sanity checks on the data we extracted
    if len(board["ports"]) != len(board["bitmasks"]):
        raise RuntimeError("Number of ports in %s doesn't match bitmask count - %d vs %d" % (board["id"], len(board["ports"]), len(board["bitmasks"])))
    if len(board["ports"]) != int(board["macros"]["NUM_DIGITAL_PINS"]):
        raise RuntimeError("Number of ports in %s doesn't match number of digital pins reported in header" % (board["id"], len(board["ports"]), board["macros"]["NUM_DIGITAL_PINS"]))


HEADER_PREFIX = """ /*
 *
 * Header for high performance Arduino Digital I/O
 *
 * Automatically generated from the Arduino library setup (boards.txt & pins_arduino.h)
 *
 * This header is a derived work of the Arduino microcontroller libraries, which are
 * licensed under LGPL. Although as a header file it is not bound by the same usage
 * clauses as the library itself (see "3. Object Code Incorporating Material from
 * Library Header Files.)"
 */

#ifndef _DIGITALIO_PERFORMANCE
#define _DIGITALIO_PERFORMANCE

"""

HEADER_SUFFIX = """

#ifndef _DIGITALIO_MATCHED_BOARD
#warning "This header's Arduino configuration heuristics couldn't match this board configuration. No fast I/O is available. The header may be out of date."

#define pinModeFast pinMode
#define digitalWriteFast digitalWrite
#define digitalReadFast digitalRead

#endif
#undef _DIGITALIO_MATCHED_BOARD
#endif
"""

BOARD_TEMPLATE = """
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

def generate_header_file(boards, identifying_keys, output):
    """
    Write a header file with fast inlined methods for all the board types
    """
    output.write(HEADER_PREFIX)
    for board in boards:
        # Work out the macro conditional
        ifdef_clause = []
        for key in identifying_keys:
            # all the +0 nonsense here is to detect defined-but-empty macros
            # (Arduino-mk inserts them)
            if key in board["macros"]:
                ifdef_clause.append("defined(%(key)s) && (%(key)s+0) == %(value)s" %
                                    {"key": key, "value": board["macros"][key]})
            else:
                ifdef_clause.append("(!defined(%s) || !(%s+0))" % (key,key))
        ifdef_clause = " && ".join(ifdef_clause)

        # Build up the macro conditionals
        digitalwrite_clause = ""
        digitalread_clause = ""
        pinmode_clause = ""
        for number,port,bitmask in zip(range(len(board["ports"])),
                                  board["ports"],
                                  board["bitmasks"]):
            digitalwrite_clause += DIGITALWRITE_TEMPLATE % locals() + "\n"
            digitalread_clause += DIGITALREAD_TEMPLATE % locals() + "\n"
            pinmode_clause += PINMODE_TEMPLATE % locals() + "\n"

        output.write(BOARD_TEMPLATE % dict(locals(), **board))
    output.write(HEADER_SUFFIX);


if __name__ == "__main__":
    main()
