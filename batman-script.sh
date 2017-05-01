#!/bin/bash

sudo modprobe batman-adv
sudo ifconfig wlan0 mtu 1528;
sudo ifconfig wlan0 down;
sudo iwconfig wlan0 mode ad-hoc essid swarm-mesh ap 02:12:34:56:78:90 channel 1;
sudo batctl if add wlan0;
sudo ifconfig wlan0 up;
sudo ifconfig bat0 up;
sudo ifconfig bat0 192.168.2.1;
