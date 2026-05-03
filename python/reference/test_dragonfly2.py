#
# This file is a command-module for Dragonfly.
# (c) Copyright 2008 by Christo Butcher
# Licensed under the LGPL, see <http://www.gnu.org/licenses/>
#

"""
Command-module loader for WSR
=============================
This script can be used to look Dragonfly command-modules 
for use with Window Speech Recognition.  It scans the 
directory it's in and loads any ``_*.py`` it finds.
"""

import os.path
import sys

import six

from dragonfly import get_engine, get_current_engine, BasicRule, Grammar, Alternative, Literal, Text
from dragonfly.loader import CommandModuleDirectory
from dragonfly.log import setup_log

#---------------------------------------------------------------------------
# Set up basic logging.

setup_log()
# logging.getLogger("compound.parse").setLevel(logging.INFO)


# Voice command rule combining spoken form and recognition processing.
class ExampleRule(BasicRule):
    element = Alternative((
        Literal("Orange", value=Text("1")),
        Literal("Stop", value=Text("0"))
        ))
    def _process_recognition(self, node, extras):   # Callback when command is spoken.
        print("Voice command spoken.")

#---------------------------------------------------------------------------
# Main event driving loop.


def main():
    global engine
    try:
        path = os.path.dirname(__file__)
    except NameError:
        # The "__file__" name is not always available, for example
        #  when this module is run from PythonWin.  In this case we
        #  simply use the current working directory.
        path = os.getcwd()
        __file__ = os.path.join(path, "dfly-loader-wsr.py")

    # Initialize and connect the engine.
    # Set any configuration options here as keyword arguments.
    engine = get_current_engine()
    if engine == None:
        engine = get_engine("sapi5inproc")
        # Load grammars.
        grammar = Grammar("example grammar")                # Create a grammar to contain the command rule.
        grammar.add_rule(ExampleRule())                     # Add the command rule to the grammar.
        grammar.load()                                      # Load the grammar.

    engine.connect()

    # Define recognition callback functions.
    def on_begin():
        print("Speech start detected.")

    def on_recognition(words):
        global engine
        message = u"Recognized: %s" % u" ".join(words)

        # This only seems to be an issue with Python 2.7 on Windows.
        if six.PY2:
            encoding = sys.stdout.encoding or "ascii"
            message = message.encode(encoding, errors='replace')
        print(message)
        engine.disconnect()

    def on_failure():
        print("Sorry, what was that?")

    # Recognize from WSR in a loop.
    try:
        engine.do_recognition(on_begin, on_recognition, on_failure)
    except KeyboardInterrupt:
        pass

    del engine


if __name__ == "__main__":
    main()