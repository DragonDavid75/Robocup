#!/bin/bash
# to be called at boot. 
# Replaces hostname with name in <robotname>.
# Get old and new hostname
hn=`hostname`
# echo "renaming" $hn "again" >/home/local/aaa.txt
# read new name in the file robotname
f="/home/local/svn/log/robotname"
cd /home/local/svn/log
date >> rename_info.txt
if [ -f $f ];
then # file exist
  # get name in file to variable nn
  nn=$(tr -d '\r\n' < "$f")
  # and length of this name
  echo "found name " $nn
  yy=$(expr length $nn)
  echo "length of" $nn "in" $f "is" $yy
  if [ $yy -gt 2 ];
  then # length is longer than 2 characters
    if [ $hn != $nn ]; 
    then # replace old hostname with new
      #echo new name $nn, so rename host from $hn
      echo New name is $nn - so rename host from $hn
      echo New name is $nn - so rename host from $hn >> rename_info.txt
      hostnamectl set-hostname $nn
      # sed -i "s/$hn/$nn/g" /etc/hostname
      # seems like /etc/hosts is not updated, but /etc/hostname is
      sed -i "s/$hn/$nn/g" /etc/hosts
    else
      # just add reboot date to file
      echo Same hostname $hn - all is fine.
      echo Same hostname $hn - all is fine. >> rename_info.txt
    fi
  fi
else
  echo "File >" $f "< not found." >> rename_info.txt
fi
# listen for MQTT power off
sleep 10
/home/local/svn/robobot/off_by_mqtt/build/off_by_mqtt 2>>off_by_mqtt2.txt >>off_by_mqtt1.txt &
# test if we are at DTU
sleep 10
ssid="DTUdevice"
echo Test access to SSID "$ssid" >> rename_info.txt
aa=$(nmcli dev wifi | grep "$ssid")
aal=$(expr length "$aa")
# echo "length of $aa is $aal"
if [ "$aal" -gt 20 ];
then
   echo SSID $ssid is available
   # trying to ping default router using 'ip r' to find router IP
   ping -q -t 1 -c 1 `ip r | grep default | cut -d ' ' -f 3` > /dev/null
   if [ "$?" -ne 0 ];
   then
     echo We are not connected, trying to connect to "$ssid" >> rename_info.txt
     nmcli device wifi connect "$ssid" password robots4ever ifname wlan0
   else
     echo We \("$HOSTNAME"\) are connected to "$ssid" >> rename_info.txt
   fi
else
   echo no $ssid SSID found >> rename_info.txt
fi
#iwconfig 2>/dev/null >> rename_info.txt
exit 0
