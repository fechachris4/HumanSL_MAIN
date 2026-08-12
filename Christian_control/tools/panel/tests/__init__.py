"""Hardware-free tests for the panel's Python side.

Run from the repository root:

    python3 -m unittest discover -s Christian_control/tools/panel/tests -t .

Nothing here starts a real binary, opens a socket or needs a robot. The
session tests launch a fake session script written by the test itself.
"""
