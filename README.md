# Solbourne Series 5E for MAME

A MAME driver for the Solbourne Series 5E workstation, written from scratch.
MAME did not have this machine.

The Series 5E is a 1991 SPARC workstation built around a Cypress CY7C601 IU at
40.1 MHz, a Solbourne KBus interconnect and a Solbourne MMU that is loaded by
software. It runs Sun software but it is not a Sun clone inside.

## Status

The machine boots OS/MP 4.1C Export from a hard disk, runs the whole multiuser
startup and reaches the login prompt. Root logs in, the file systems pass
`fsck`, and data written to disk reads back byte for byte after a reboot.

Emulated: the CPU with its FPU, the MMU with both translation windows, memory
with its ECC, the WD33C93A SCSI controller with hard disks, the real time
clock and both Z8530 serial controllers. The console is serial port A.

Not emulated: the LANCE Ethernet controller, the frame buffer and the
Solbourne keyboard.

Known problems:

- The machine needs a device at SCSI target 1. With only one disk it panics on
  a page fault as soon as `init` hands control to the boot scripts. The cause
  is not understood yet and does not look like a SCSI problem.
- The kernel prints `WARNING: Kbus and/or VMEbus SYSFAIL asserted` and
  `WARNING: TOD clock not initialized` on every boot, and carries on.

## Layout

    src/mame/skeleton/solbourne.cpp   the driver
    patches/                          changes to the rest of the MAME tree
    tools/console.py                  a terminal for the emulated serial console
    tools/stop.lua                    clean shutdown for an autoboot script

## Building

Copy the driver into a MAME source tree, apply the patches and build the
driver on its own:

    cp src/mame/skeleton/solbourne.cpp <mame>/src/mame/skeleton/
    cd <mame>
    patch -p1 < <this>/patches/0001-sparc-fp-queue-and-cpu-id.patch
    patch -p1 < <this>/patches/0002-wd33c9x-restart-sequencer-on-fifo-edge.patch
    make SUBTARGET=solb SOURCES=src/mame/skeleton/solbourne.cpp

## The patches

Both fix bugs in existing MAME devices that this machine runs into. They are
kept here as patches on purpose; they are not submitted anywhere.

**0001, SPARC.** Two problems in `sparcv7`. `STDFQ` never retired its
instruction, so a kernel that stores the floating point queue spins on it
forever. And the device had no floating point queue at all, which OS/MP needs:
without one its trap handler cannot tell a pending exception from a sequence
error. The same patch lets a driver set the implementation and version fields
of the PSR, which the OS/MP kernel checks before it will boot.

**0002, WD33C93A.** `dma_r()` and `dma_w()` did not restart the sequencer when
the FIFO went from full to not full, or from empty to not empty, the way the
programmed I/O path does. The chip stalls for good if the host pauses long
enough to fill or drain it.

Note that the second one lets the controller ask for data from inside `dma_r`
and `dma_w`, so a DMA engine that calls them from its own loop can re-enter
itself and skip bytes. The driver here guards against that with a lock.

## Running

The driver takes the system disk as `-hard1` and a second SCSI device as
`-hard2`. The console comes out of serial port A:

    mame -rompath <roms> sols5e -hard1 system.hd -hard2 install.hd -ttya pty

At the `ROM>` prompt, `boot sd.si(,0,)vmunix` starts the system, and `-s`
after it gives single user.

## License

BSD-3-Clause, the same as the MAME source it is built against. See LICENSE.
