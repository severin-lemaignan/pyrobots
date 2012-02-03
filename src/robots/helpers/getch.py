class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self, msg = None):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            if msg:
                 print(msg) 
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
        if ord(ch) == 3: #Ctrl+C
            raise KeyboardInterrupt()
        return ch

getch = _GetchUnix()
