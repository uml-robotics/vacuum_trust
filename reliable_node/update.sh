#!/bin/bash
REPO=reliablenode
ROBOTS=(neato.lan roomba500.lan discovery.lan bender.lan dirtdog.lan)

curl=`which curl`

case $1 in
    run)
        for robot in ${ROBOTS[@]}; do
            echo "Updating $robot"
            $curl --max-time 10 $robot:8000/$REPO
        done
        echo "Finished!"
        ;;
    install)
        git config alias.xpush '!git push $1 $2 && ./update.sh run'
        ;;
    *)
        echo "Usage: update.sh {install|run}"
        ;;
esac
