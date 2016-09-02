# Toolchain Installation

On Ubuntu 16.04 (Xenial) add the ARM embedded team's PPA and install
their toolchain for the latest awesome cortex-m7 support (including FPU):
```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
```

Unfortunately, the version of OpenOCD that comes with Ubuntu 16.04 is not
quite awesome enough to include support for STM32F7. So we'll need to build
it from source. I prefer to install it to /usr/local to simplify usage. The
following instructions work on my box, but I've got an absurd amount of
packages installed; you may need to add some Ubuntu/Debian packages if the
bootstrap or configure scripts complain.

```
sudo apt-get install libhidapi-dev
cd ~
git clone http://repo.or.cz/r/openocd.git
cd openocd
./bootstrap
./configure --enable-stlink --enable-ftdi --enable-cmsis-dap --prefix=/usr/local
make -j8
sudo make install
```

# Organization

The "common" directory contains a peripheral library for this board. Those
sources are linked into the various applications, which are siblings of the
"common" directory. At time of writing, that consists solely of a "blink"
program and a "console" program which just prints hello.

# Compiling Stuff

To build an application, just go into its directory and type `make`. That will
build the common library, the application sources, and put the resulting
build products in a `bin` subdirectory of the application directory.

# Flash Programming Stuff

To flash an application, just go into its directory and type `make program`.

# Debugging Stuff

The most useful debugging tool of all time is still `printf()`. Using the
ST-LINK on the Nucleo boards is nice because the ST-LINK microcontroller
acts as both a JTAG adapter and a virtual serial port over the same USB cable,
so you have less mess on your desk. You can view the serial console using
the little `console` program in the source tree:

```
cd [ROOT OF GIT CLONE OF THIS REPOSITORY]
cd software/console
make
bin/console /dev/ttyACM0
```

The last line assumes that you don't have any other USB CDC devices attached
to your machine; you may need to use a different /dev/ttyACM in that case.

When `printf()` is not sufficient, the next tool to reach for is usually `gdb`.
You'll need two consoles. In the first console, type `make program` to ensure
that you have flashed the latest binary, and then type `make gdb_server` to
attach to it on the target. Then, in a second console, type `make gdb` to
launch GDB and halt the target. Then you can set breakpoints, run the program,
and poke around as usual.
