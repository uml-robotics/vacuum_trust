#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONFDIR=/etc/supervisor/conf.d
cd $DIR
mkdir $HOME/logs
sudo apt-get install supervisor
[ ! -f $CONFDIR/robotcontroller.conf ] && sudo ln -s $DIR/robotcontroller.conf $CONFDIR/robotcontroller.conf
[ ! -f $CONFDIR/hostutils.conf ] && sudo ln -s $DIR/hostutils.conf $CONFDIR/hostutils.conf
sudo systemctl restart supervisor.service
