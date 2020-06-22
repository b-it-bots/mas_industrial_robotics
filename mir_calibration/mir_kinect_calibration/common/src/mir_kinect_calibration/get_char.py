class _GetChar:

    """
    Gets a single character from standard input, does not echo to the screen.
    Recipe from: http://code.activestate.com/recipes/134892/
    """

    def __init__(self):
        try:
            self.impl = _GetCharUnix()
        except ImportError:
            self.impl = _GetCharWindows()

    def __call__(self):
        return self.impl()


class _GetCharUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetCharWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt

        return msvcrt.getch()


get_char = _GetChar()
