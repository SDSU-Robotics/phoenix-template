#!/bin/bash

sudo slcand -o -c -s8 /dev/ttyACM1 can0

interface=can0
if [ $# -gt 0 ]; then
    interface=$1
fi

sudo ifconfig $interface up
sudo ifconfig $interface txqueuelen 10000