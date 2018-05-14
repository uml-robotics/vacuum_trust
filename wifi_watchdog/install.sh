sudo apt-get install wicd-cli
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
sudo cp $DIR/10-wifiwatchdog.conf /etc/rsyslog.d/10-wifiwatchdog.conf
[ $( sudo crontab -l | grep "wifi_watchdog" -c) -eq 0 ] && (sudo crontab -l ; echo "*/2 * * * * $DIR/recon.sh") | sudo crontab -
sudo systemctl restart rsyslog.service
