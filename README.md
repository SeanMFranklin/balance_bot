# mbot-omni-firmware
Firware to run on the RPi Pico for the MBot Omni.

## Installation

After cloning the repo, cd into the mbot-omni-firmware directory.
Then, run the setup script:

```bash
./setup.sh
```

Which will install dependencies (requires sudo password) and initialize the submodules.

If setup.sh is not executable by default, do the following to enable it:

```bash
sudo chmod +x setup.sh
```

### Possible Issue
If you come across an error that says that you cannot clone `lib/pico-sdk`, follow these steps:
```bash
cd lib
rm -rf pico-sdk
git clone git clone git@github.com:rob102-staff/pico-sdk.git
cd ..
sudo ./setup.sh
cd lib/pico-sdk
git checkout master
git pull
cd ../../
mkdir build
cd build
cmake ..
make -j4
```

## Building

Build as follows:
```bash
mkdir build
cd build
cmake ..
make
```

## Installing picotool
NOTE: We should add this to the setup.sh script, as it can be done as soon as the pico-sdk is cloned
```bash
wget https://github.com/raspberrypi/picotool/archive/refs/tags/1.1.1.zip
unzip 1.1.1.zip
cd picotool-1.1.1
mkdir build && cd build
export PICO_SDK_PATH=~/mbot_ws/mbot_firmware/lib/pico-sdk
cmake ..
make
sudo make install
```

## Installing openocd and gdb-multiarch
NoteL We should add this to the setub.sh script
```bash
sudo apt-get install libftdi-dev gdb-multiarch
git clone https://github.com/raspberrypi/openocd.git --recursive --branch rp2040 --depth=1
cd openocd
./bootstrap
./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio
make -j4
sudo make install
```

## Uploading firmware to the pico
There are a few ways to upload firmware, using the bootload partition on the pico, using picotool or using the upload.sh script and openocd.

### Bootloader
Plug in the pico while holding the BOOTLOAD button.  The pico will be mounted as a drive.  Copy the mbot.uf2 file over to the drive.

### picotool
Plug in the pico while holding the BOOTLOAD button.  Run:
```bash
picotool load build/src/mbot.uf2
picotool reboot
```

### upload.sh and openocd
Run the upload script which uses openocd.  This does not require puting the Pico into bootloader mode.  You must have the SWD wires (SWDIO, SWGND, and SWCLK) connected to GPIO 24 (Pin 18), GND (Pin 20) and GPIO 25 (Pin 22) on the Raspberry Pi.  Note, when using the upload script and openocd, you upload the .elf firmware file, not the .uf2 firmware file, they are just a different format, but the same firmware.
 ```bash
 upload.sh build/src/mbot.elf
 ```



