#!/bin/bash

#initialisation of Virtual CAN interface and CANopen nodes for 
#testing of socket connections.

#setting up CAN0
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up

echo - > storage/od4_storage
echo - > storage/od4_storage_auto