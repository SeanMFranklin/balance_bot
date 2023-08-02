#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Usage: $0 <uf2_file>"
  exit 1
fi

UF2_FILE=$1

### To use this script for upload, plug  3 pin cable into the Run/Rst/Btld port
### and to pins 19, 21 & 23 on the jetson nano
### This should work on the Pi, but the pins may be different

#GPIO Numbers
BTLD_PIN=16
RUN_PIN=18

#setup GPIO pins
echo $BTLD_PIN > /sys/class/gpio/export
echo $RUN_PIN > /sys/class/gpio/export
sleep 0.1
echo out > /sys/class/gpio/gpio$BTLD_PIN/direction
echo out > /sys/class/gpio/gpio$RUN_PIN/direction
sleep 0.1
echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
sleep 0.1

#Toggle to bootloader
echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
sleep 0.1
echo 0 > /sys/class/gpio/gpio$BTLD_PIN/value
sleep 0.5
echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
sleep 1

sudo picotool load $UF2_FILE
sleep 0.5
echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
sleep 0.5
sudo picotool reboot

echo in > /sys/class/gpio/gpio$BTLD_PIN/direction
echo in > /sys/class/gpio/gpio$RUN_PIN/direction
echo $BTLD_PIN > /sys/class/gpio/unexport
echo $RUN_PIN > /sys/class/gpio/unexport