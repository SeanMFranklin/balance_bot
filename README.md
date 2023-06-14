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

## Using GDB on the Pico

As long as the SWD wires are connected to the Raspberry Pi (see above section on main branch), you can use GDB on the Pico to help debug your current firmware or test program.

Note: make sure that the elf file was built using the `-DCMAKE_BUILD_TYPE=Debug` flag. If the file was built for Release, the optimization means most things won't be visible when you use the print command in gdb later.

Build as normal, then upload using:
```bash
upload.sh path/to/elf/file.elf
```

In one window, run:
```bash
debug.sh path/to/elf/file.elf
```

Which starts up the GDB server and blocks this terminal until you Ctrl+C to quit the server. In another terminal, run:

```bash
debug_attach.sh path/to/elf/file.elf
```

Which attaches to the server and stops the program at the start of the main function, from where you can start debugging with GDB commands.

TODO: If we really have time, try to get this working in VSCode.