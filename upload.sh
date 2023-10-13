#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Usage: $0 <uf2_file>"
  exit 1
fi

UF2_FILE=$1

# GPIO Numbers, 
# on Jetson, pin 7 is gpio216 and pin 11 is gpio50
# on RPi pin 7 is gpio4, pin 11 is gpio17
if grep -q "Raspberry Pi" /proc/device-tree/model; then
  echo "Detected Raspberry Pi, entering bootload mode..."
  BTLD_PIN=4
  RUN_PIN=17
elif grep -q "NVIDIA Jetson" /proc/device-tree/model; then
  echo "Detected NVIDIA Jetson, entering bootload mode..."
  BTLD_PIN=50
  RUN_PIN=216
else
  echo "ERROR: Unknown hardware!"
  exit 1
fi

# setup GPIO pins
echo $BTLD_PIN > /sys/class/gpio/export
echo $RUN_PIN > /sys/class/gpio/export
sleep 0.1
echo out > /sys/class/gpio/gpio$BTLD_PIN/direction
echo out > /sys/class/gpio/gpio$RUN_PIN/direction
sleep 0.1
echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
sleep 0.1

# Toggle to bootloader
echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
sleep 0.1
echo 0 > /sys/class/gpio/gpio$BTLD_PIN/value
sleep 0.5
echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
sleep 1

# Upload code
sudo picotool load $UF2_FILE
sleep 0.5
echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
sleep 0.5
sudo picotool reboot

# Make pins High-Z and unexport
echo in > /sys/class/gpio/gpio$BTLD_PIN/direction
echo in > /sys/class/gpio/gpio$RUN_PIN/direction
echo $BTLD_PIN > /sys/class/gpio/unexport
echo $RUN_PIN > /sys/class/gpio/unexport