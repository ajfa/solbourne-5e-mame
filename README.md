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

One processor. The real machine was SMP and OS/MP is genuinely multiprocessor,
but the driver puts a single CY7C601 in KBus slot 3 and the PROM reports
`One CPU (slot 3)`.

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

## Building the system disk

The disk this boots from is not here: it is Solbourne's OS/MP, and it is made by
running their own installer inside the emulator.

**The medium.** OS/MP 4.1C Export comes on tape, and what the installer reads here
is that tape dumped into a disk image. The layout, measured: a Sun label for a
Seagate ST11200, 1868 cylinders, 15 heads, 73 sectors, a single `a` partition of
175 MB, and inside it a plain UFS holding the tape's files, `TOC`,
`Install.Series5`, `Miniusr`, `Root.tar`, `Usr.tar`, `Kvm.Series5.tar` and the
optional sets. The dump is not redistributed here.

**The installer will not read a disk.** It offers tape, CD-ROM or network and
nothing else, so the medium is attached as the second SCSI device and answered for
as a tape. Its menu fields come pre-filled with whatever was typed before, and the
old value has to be cleared before typing over it.

**Three things have to be done to the new disk afterwards**, or the machine does not
come up the way it should:

- Remove `kvm/stand/dg`. With that file in place the PROM starts the diagnostic by
  itself instead of dropping to `ROM>`.
- Put `stty pass8 < /dev/console > /dev/console` at the top of `/etc/rc.boot` and of
  `/etc/rc`, and use the `cons8` getty entry in `/etc/ttytab`. The next section says
  why.
- Leave a device at SCSI target 1. With a single disk the kernel takes a page fault
  as soon as `init` hands control to the boot scripts, which is the first of the
  known problems above.

The system behind this README answers to `root`, calls itself `solbourne`, has no
network configured and runs in GMT. From power on to the login prompt is about
forty five seconds.

## The console line

The PROM drives serial port A at 9600 8N1. Once `init` takes over, the SunOS
tty driver puts the line in seven bits with even parity, so every byte from
there to the login prompt arrives with the eighth bit set: a terminal wired for
8N1 shows `s` as 0xF3. Two changes on the installed system settle it:

- In `/etc/ttytab`, use the `cons8` getty entry for the console. The install
  leaves it commented out one line above the `std.9600` one it does use.
- At the top of `/etc/rc`, add `stty pass8 < /dev/console > /dev/console`.

Do not put that line in `/etc/rc.boot`. It runs before `/usr` is mounted and
`stty` lives in `/usr/bin`, so all it does is print `stty: not found`.

The console device itself comes from an EEPROM variable. The system board ID
PROM ships with `CONSOLE=zs()`, which is the serial port.

## What a frame buffer would take

SunView runs on the on-board frame buffer, and the guest side is ready for
it. The PROM lists the board in its slot table:

    7    G0   AG    BW20 Monochrome Frame Buffer

the kernel carries a `bwtwo` driver with a Solbourne id string in it, and an
installed system already has `/dev/bwtwo0`, `/dev/kbd` and `/dev/mouse`.

What the driver has today:

- The frame buffer memory, 256 KB, and the VIDMAP register at offset 0x200000
  of the system board slot, which maps that memory into a KBus space of its
  own. Reads and writes reach it.
- The second Z8530, the keyboard and mouse pair of ports, instantiated with its
  interrupt wired to line b. Nothing is attached to it.

What is missing:

- A screen device and a screen update that draws the frame buffer memory as a
  monochrome bitmap. The geometry is not confirmed. 256 KB is a good deal more
  than the 129,600 bytes a 1152x900 mono display needs, so the decode is worth
  measuring against the kernel's own `bwtwo` driver rather than assumed.
- Whatever control registers the BW20 has besides plain memory. A frame buffer
  of this generation usually has at least a video enable and a vertical retrace
  interrupt. Neither has been located yet.
- A keyboard and a mouse on that second Z8530. MAME has Sun keyboard and mouse
  devices to hang there. The ID PROM carries `KBD_LAYOUT=0` and a full key
  translation table, so the PROM expects a local keyboard.

The screen alone is a good first step, because it checks itself. The kernel
prints `Changing Console to bwtwo0` when it moves the console off the serial
line, so a working screen shows the tail of the boot with no keyboard involved.

## License

BSD-3-Clause, the same as the MAME source it is built against. See LICENSE.
