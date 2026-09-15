#!/usr/bin/env python3
"""Bridge between your terminal and the pseudo terminal MAME opens.

Leaves your terminal raw, so the vt100 sequences from the guest arrive whole
and curses works. Answers only what the automatic list says.

Usage: console.py <pty path> [--log file] [--no-keyboard]
"""
import os
import select
import sys
import termios
import tty

# (what to wait for, what to answer); each one is used once
AUTOMATIC = [
    (b"ROM>", b"boot sd.si(,0,)vmunix\r"),
]

QUIT = 0x1c  # Ctrl-\


def main():
    path = sys.argv[1]
    log = None
    keyboard = True
    if "--log" in sys.argv:
        log = open(sys.argv[sys.argv.index("--log") + 1], "wb")
    if "--no-keyboard" in sys.argv:
        keyboard = False

    pty = os.open(path, os.O_RDWR | os.O_NOCTTY)
    pending = list(AUTOMATIC)
    tail = b""

    saved = None
    if keyboard and sys.stdin.isatty():
        saved = termios.tcgetattr(sys.stdin.fileno())
        tty.setraw(sys.stdin.fileno())

    try:
        while True:
            sources = [pty]
            if keyboard:
                sources.append(sys.stdin.fileno())
            ready, _, _ = select.select(sources, [], [], 1.0)

            if pty in ready:
                data = os.read(pty, 4096)
                if not data:
                    break
                os.write(sys.stdout.fileno(), data)
                if log:
                    log.write(data)
                    log.flush()
                tail = (tail + data)[-256:]
                if pending and pending[0][0] in tail:
                    os.write(pty, pending[0][1])
                    tail = b""
                    pending.pop(0)

            if keyboard and sys.stdin.fileno() in ready:
                key = os.read(sys.stdin.fileno(), 1024)
                if not key:
                    break
                if bytes([QUIT]) in key:
                    break
                os.write(pty, key)
    except (OSError, KeyboardInterrupt):
        pass
    finally:
        if saved is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, saved)
        os.close(pty)
        if log:
            log.close()


if __name__ == "__main__":
    main()
