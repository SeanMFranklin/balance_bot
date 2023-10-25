#!/bin/bash

EXPECTED_ARGS=2
E_BADARGS=65

if [ $# -ne 2 ]
then
  echo "Usage: $0 [load | run | flash | disable] <uf2_file>"
  exit $E_BADARGS
fi

OPERATION=$1
UF2_FILE=$2

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

case "$OPERATION" in
    "load"|"run"|"flash"|"disable")
        # notice that for all cases, we consdier the RUN_PIN is high
        # which set by services/mbot_start_networking.py 
        case "$OPERATION" in
            "load")
                echo "Loading action for $UF2_FILE..."
                echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 0.1
                echo 0 > /sys/class/gpio/gpio$BTLD_PIN/value
                sleep 0.5
                echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 0.5
                sudo picotool load $UF2_FILE
                sleep 0.5
                ;;
            "run")
                echo "Running action for $UF2_FILE..."
                echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 0.1
                echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
                sleep 0.5
                echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 1
                ;;
            "flash")
                echo "Flashing action for $UF2_FILE..."
                echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 0.1
                echo 0 > /sys/class/gpio/gpio$BTLD_PIN/value
                sleep 0.5
                echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 0.5
                sudo picotool load $UF2_FILE
                sleep 0.5
                echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
                sleep 0.5
                sudo picotool reboot
                ;;
            "disable")
                echo "Disable action for $UF2_FILE..."    
                echo 0 > /sys/class/gpio/gpio$BTLD_PIN/value
                sleep 0.5
                echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 1
                ;;
        esac
        ;;
    *)
        echo "'$OPERATION' is not a valid operation. Please provide one of the following operations: load, run, flash, disable"
        exit 1
        ;;
esac

# Make pins High-Z and unexport
echo in > /sys/class/gpio/gpio$BTLD_PIN/direction
echo in > /sys/class/gpio/gpio$RUN_PIN/direction
echo $BTLD_PIN > /sys/class/gpio/unexport
echo $RUN_PIN > /sys/class/gpio/unexport

