# Installation

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
