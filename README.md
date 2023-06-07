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

