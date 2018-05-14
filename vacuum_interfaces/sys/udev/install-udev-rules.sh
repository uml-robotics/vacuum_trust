#!/bin/bash
# Installs the udev rules found in this directory to /etc/udev/rules.d/.
# This script can be run from any location.
# ./install-udev-rules.sh

# Get the full path to the directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

UDEVPATH="/etc/udev/rules.d"
echo "Installing the following files:"
for rulefile in $(find $DIR/* -name "*.rules"); do
    # FILENAME="90-bluegiga.rules"
    FILENAME=`basename $rulefile`
    echo $FILENAME
    sudo cp $rulefile $UDEVPATH/$FILENAME
    sudo chmod 644 $UDEVPATH/$FILENAME
    sudo chown root $UDEVPATH/$FILENAME
    sudo chgrp root $UDEVPATH/$FILENAME
done

echo "Reloading udev rules. Please restart or replugin any devices"
sudo udevadm control --reload-rules
