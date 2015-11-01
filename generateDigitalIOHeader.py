#!/usr/bin/python2
# -*- coding: utf-8 -*-

"""
Code generation script to build digitalIOPerformance.h from the Arduino
boards.txt & pins_arduino.h files.

You shouldn't need to rerun this unless you have a new version of
Arduino installed, or you're using different boards to the default
installation.

See the README file for more details.

Copyright (c) 2013 Angus Gratton. Script licensed under the GNU Lesser
General Public License, version 2 or above.

"""

import os, sys, re, subprocess, glob

# TODO: command line options
ARDUINO_PATH = sorted(glob.glob(os.path.expanduser("~") + "/.arduino15/packages/arduino/hardware/avr/*"))[-1]
MENU_KEY = 'menu.cpu'


class PreprocessorError(Exception):
    pass


def main():
    boards = []
    for board in extract_boards():
        print "Processing", board['id']
        try:
            add_variant_macros(board)
            boards.append(board)
        except PreprocessorError as exc:
            print "No support for board", board['id'], "- can't add_variant_macros, sorry..."
            print "\t%s" % exc

    find_unambiguous_macros(boards)
    identifying_keys = find_unique_macro_keys(boards)
    boards = merge_matching_boards(boards, identifying_keys)
    outboards = []
    for board in boards:
        try:
            extract_portnames_pins(board)
            outboards.append(board)
        except RuntimeError as exc:
            print "No support for board", board['id'], "- can't extract_portnames_pins, sorry..."
            print "\t%s" % exc

    with open("digitalIOPerformance.h", "w") as output:
        generate_header_file(outboards, identifying_keys, output)

def expand_board(board):
    """Expand a board into subboards"""
    common = {}
    sub_boards = {}
    for key, value in board.iteritems():
        if key == 'id':
            continue
        if key.startswith(MENU_KEY + '.'):
            path = key.split('.', 3)
            if len(path) == 3:
                path.append('name')
            sub_id = board['id'] + '/' + path[2]
            if sub_id not in sub_boards:
                sub_boards[sub_id] = {'id': sub_id}
            sub_boards[sub_id][path[3]] = value
        else:
            common[key] = value
    if 'build.mcu' in common:
        del common['build.mcu']
    for sub_board in sub_boards.itervalues():
        name = sub_board['name']
        sub_board.update(common)
        sub_board['name'] = sub_board['name'] + '/' + name

    return sub_boards.values()

def extract_boards():
    """ Parse the Arduino boards file and return a list of all the boards,
    with each board as a dictionary containing all their board-specific
    key-value pairs
    """
    data = [i for i in open(os.path.join(ARDUINO_PATH, "boards.txt"), 'rb').readlines()
            if not i.startswith('#') and '=' in i]
    boards = {}
    for line in data:
        key, value = line.strip().split('=', 1)
        if key == MENU_KEY:
            if value != 'Processor':
                raise RuntimeError("Unsupported menu by %r!" % value)
            continue
        board_id, path = key.split('.', 1)
        if board_id not in boards:
            boards[board_id] = {'id': board_id}
        board = boards[board_id]
        assert path not in board
        board[path] = value
    xboards = []
    for board_id in sorted(boards):
        board = boards[board_id]
        has_sub = False
        for key in board:
            if key.startswith(MENU_KEY + '.'):
                has_sub = True
                break
        if not has_sub:
            xboards.append(board)
        else:
            xboards.extend(expand_board(board))
    return xboards

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
    # dev_null = open(os.path.devnull, 'r+')
    args = args + additional_args + [ source_path ]
    proc = subprocess.Popen(args, stdout=subprocess.PIPE)
    stdout, stderr = proc.communicate()
    if proc.returncode != 0:
        sys.stderr.write(stdout or '')
        sys.stderr.write(stderr or '')
        print repr(board)
        raise PreprocessorError("Error running preprocessor (%r)" % (args,))
    return stdout

def add_variant_macros(board):
    """
    Run the pin_arduinos.h header for this board through the preprocessor,
    extract all defined macros and attach them to the board dict under key
    'macros'
    """
    board["pin_path"] = os.path.join(ARDUINO_PATH, "variants/%s/pins_arduino.h" % (board["build.variant"]))
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
        for key, value in board["macros"].items():
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
        duplicates = list(dup for dup in boards if dup != board)
        for dup in duplicates:
            # skip anything that's already unique as per existing keys
            identified = False
            for key in identifying_keys:
                identified = identified or board["macros"].get(key, "") != dup["macros"].get(key, "")
            if identified:
                continue

            # find something unique about this duplicate
            uniques = set(board["macros"].items()) ^ set(dup["macros"].items())
            uniques = [ key for key, value in uniques ]
            # find the least selective key in the uniques
            def selectiveness(key):
                return len([d for d in duplicates if (d["macros"].get(key, "") == board["macros"].get(key, ""))])
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
        return str([ board["macros"].get(key, "") for key in identifying_keys ])

    # Merge together any boards with identical keys (making composite names & ids)
    unique_boards = []
    for board in boards:
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
    def extract_array(array_name):
        """ Match a multiline C array with the given name (pretty clumsy.) """
        content = re.search(array_name + ".+?\{(.+?)}", output, re.MULTILINE | re.DOTALL).group(1)
        return [ e.strip() for e in content.split(",") if len(e.strip()) ]

    output = run_preprocessor(board)
    board["ports"] = extract_array("digital_pin_to_port_PGM")
    # strip P prefix, ie PD becomes D
    board["ports"] = [ e[1] if e[0] == "P" else None for e in board["ports"] ]

    board["bitmasks"] = extract_array("digital_pin_to_bit_mask_PGM")

    pin_to_timer = extract_array("digital_pin_to_timer_PGM")
    board["timers"] = [ p if p != "NOT_ON_TIMER" else None for p in pin_to_timer ]

    # do some sanity checks on the data we extracted
    if len(board["ports"]) != len(board["bitmasks"]):
        raise RuntimeError("Number of ports in %s doesn't match bitmask count - %d vs %d" % (board["id"], len(board["ports"]), len(board["bitmasks"])))
    if "NUM_DIGITAL_PINS" not in board["macros"]:
        raise RuntimeError("No digital pins count for %s in the header, ports = %i" % (board["id"], len(board["ports"])))
    if len(board["ports"]) != int(board["macros"]["NUM_DIGITAL_PINS"]):
        raise RuntimeError("Number of ports in %s doesn't match number of digital pins reported in header" % (board["id"], len(board["ports"]), board["macros"]["NUM_DIGITAL_PINS"]))

DIGITALWRITE_TEMPLATE = """
  else if(pin == %(number)s && value) PORT%(port)s  |= %(bitmask)s;
  else if(pin == %(number)s && !value) PORT%(port)s &= ~%(bitmask)s;
""".lstrip("\n")

PINMODE_TEMPLATE = """
  else if(pin == %(number)s && mode == INPUT) {
    DDR%(port)s &= ~%(bitmask)s;
    PORT%(port)s &= ~%(bitmask)s;
  } else if(pin == %(number)s && mode == INPUT_PULLUP) {
    DDR%(port)s &= ~%(bitmask)s;
    PORT%(port)s |= %(bitmask)s;
  } else if(pin == %(number)s) DDR%(port)s |= %(bitmask)s;
""".lstrip("\n")

DIGITALREAD_TEMPLATE = """
  else if(pin == %(number)s) return PIN%(port)s & %(bitmask)s ? HIGH : LOW;
""".lstrip("\n")

TIMER_TEMPLATE = """
  else if(pin == %(number)s) %(timer_reg)s &= ~_BV(%(timer_bit)s);
""".lstrip("\n")

ISPWM_TEMPLATE = """
  if(pin == %(number)s)
    return true;
""".lstrip("\n")

PORT_TEST_TEMPLATE = """
  if(pin == %(number)s)
    return _SFR_IO_REG_P(%(port)s);
""".lstrip("\n")

# Lookup table from the timer specifications given in pins_arduino.h
# to the actual timer register bits
#
# This is equivalent of the case statement in wiring_digital.c, so if that
# ever changes this needs to change too. :(
TIMER_LOOKUP = {
		"TIMER1A": ("TCCR1A", "COM1A1"),
		"TIMER1B": ("TCCR1A", "COM1B1"),
		"TIMER2":  ("TCCR2" , "COM21" ),
		"TIMER0A": ("TCCR0A", "COM0A1"),
		"TIMER0B": ("TCCR0A", "COM0B1"),
		"TIMER2A": ("TCCR2A", "COM2A1"),
		"TIMER2B": ("TCCR2A", "COM2B1"),
		"TIMER3A": ("TCCR3A", "COM3A1"),
		"TIMER3B": ("TCCR3A", "COM3B1"),
		"TIMER3C": ("TCCR3A", "COM3C1"),
		"TIMER4A": ("TCCR4A", "COM4A1"),
		"TIMER4B": ("TCCR4A", "COM4B1"),
		"TIMER4C": ("TCCR4A", "COM4C1"),
		"TIMER4D": ("TCCR4C", "COM4D1"),
		"TIMER5A": ("TCCR5A", "COM5A1"),
		"TIMER5B": ("TCCR5A", "COM5B1"),
		"TIMER5C": ("TCCR5A", "COM5C1"),
}

def generate_header_file(boards, identifying_keys, output):
    """
    Write a header file with fast inlined methods for all the board types
    """
    with open("components/board_template.cpp") as template:
        BOARD_TEMPLATE = template.read()
    with open("components/header.cpp") as header:
        output.write(header.read())
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
                ifdef_clause.append("(!defined(%s) || !(%s+0))" % (key, key))
        ifdef_clause = " && ".join(ifdef_clause)

        # Build up the macro conditionals
        digitalwrite_clause = ""
        digitalread_clause = ""
        pinmode_clause = ""
        timer_clause = ""
        ispwm_clause = ""
        direction_clause = ""
        output_clause = ""
        input_clause = ""
        for number, port, bitmask, timer in zip(range(len(board["ports"])),
                                             board["ports"],
                                             board["bitmasks"],
                                             board["timers"]):
            if port is not None:
                digitalwrite_clause += DIGITALWRITE_TEMPLATE % locals()
                digitalread_clause += DIGITALREAD_TEMPLATE % locals()
                pinmode_clause += PINMODE_TEMPLATE % locals()
                if timer is not None:
                    timer_reg, timer_bit = TIMER_LOOKUP[timer]
                    timer_clause += TIMER_TEMPLATE % locals()
                    ispwm_clause += ISPWM_TEMPLATE % locals()
                direction_clause += PORT_TEST_TEMPLATE % {"number":number,
                                                          "port":"DDR" + port}
                output_clause += PORT_TEST_TEMPLATE % {"number":number,
                                                       "port":"PORT" + port}
                input_clause += PORT_TEST_TEMPLATE % {"number":number,
                                                       "port":"PIN" + port}

        output.write(BOARD_TEMPLATE % dict(locals(), **board))
    with open("components/footer.cpp") as footer:
        output.write(footer.read())


if __name__ == "__main__":
    main()
