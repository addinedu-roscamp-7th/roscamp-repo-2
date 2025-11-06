#!/bin/bash

echo ' ===== Camera udev Rules Setting ===== '

if  [ ! -e /etc/udev/rules.d/98-camera-name-setting.rules ]; then
    echo '!!!!! rules file not exist. copy rules file !!!!!'
    sudo cp 98-camera-name-setting.rules /etc/udev/rules.d
    sudo udevadm control --reload && sudo udevadm trigger
else
    echo '!!!!! rules file already exist !!!!!'
fi