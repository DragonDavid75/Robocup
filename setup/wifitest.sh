#!/bin/bash
#wget -q --spider http://dtu.dk
#ping -c2 dtu.dk
#
# do this interface exist
# if not, then reboot (after 2 minutes)
# save result to logfole
date >>/home/local/rebootinfo.txt
ifconfig wlp58s0  >>/home/local/rebootinfo.txt
if [ $? -eq 0 ]; then
    echo "wifi device wlp58s0 is Online" >>/home/local/rebootinfo.txt
else
    echo "wifi device wlp58s0 is Offline" >>/home/local/rebootinfo.txt
    sleep 120
    echo "time to reboot" >>/home/local/rebootinfo.txt
    reboot now
fi
