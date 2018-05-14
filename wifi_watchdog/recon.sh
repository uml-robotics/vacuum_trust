#!/bin/bash
SERVER=10.0.1.1
NETWORKS='atrvnw\ 5GHz\|Pterodactyl\ 5GHz\|chimaera\ 5GHz'

ping -c9 ${SERVER} > /dev/null

if [ $? != 0 ]
then
    logger -t wifiwatchdog "No connected to network, scanning for APs"
    # echo "'${NETWORKS}'"
    if [ -f /tmp/tick ]; then
        if [ $(($(date +'%s') - `cat /tmp/tick`)) -gt 180 ]; then
            logger -t wifiwatchdog "Kicking wifi in the ass"
            /bin/systemctl stop wicd.service
            /usr/sbin/service networking stop
            /usr/sbin/rfkill block 0
            /sbin/modprobe -r mt7610u  #&& /sbin/modprobe mt7610u
            /usr/sbin/service networking start
            /usr/sbin/rfkill unblock 0
            /bin/systemctl start wicd.service
            echo $(date +'%s') > /tmp/tick
            /bin/systemctl restart networking.service
            sleep 20 
        fi
    fi
    /usr/bin/wicd-cli -y -x
    /usr/bin/wicd-cli -z -x
    /usr/sbin/rfkill unblock 0
    APS=$(/usr/bin/wicd-cli -y -S -l | grep "${NETWORKS}" | cut -f1)
    if [[ -z ${APS// } ]]; then
        logger -t wifiwatchdog "got $(($(/usr/bin/wicd-cli -y -l | wc -l) - 1)) results"
        sleep 5
        APS=$(/usr/bin/wicd-cli -y -S -l | grep "${NETWORKS}" | cut -f1)
        if [[ -z ${APS// } ]]; then
            logger -t wifiwatchdog "got $(($(/usr/bin/wicd-cli -y -l | wc -l) - 1)) results"
            logger -t wifiwatchdog "NO APS FOUND"
            /usr/bin/wicd-cli -z -c
            exit 0
        fi
    fi

    APS_AR=($APS)
    logger -t wifiwatchdog "Connecting to ${APS_AR[0]}"
    /usr/bin/wicd-cli -y -x
    /usr/bin/wicd-cli -y -n ${APS_AR[0]}  -c
    echo "$(date)" >> /home/pi/routes
    echo "$(/sbin/route)" >> /home/pi/routes
else
    logger -t wifiwatchdog "connection good"
    echo $(date +'%s') > /tmp/tick
fi
