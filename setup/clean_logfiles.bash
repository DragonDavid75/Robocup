#!/bin/bash
# clean logfiles before flash image
# teensy_interface
rm -r /home/local/svn/robobot/teensy_interface/build/log_20*
rm /home/local/svn/robobot/teensy_interface/build/out_*.txt
# mqtt_python
rm /home/local/svn/robobot/mqtt_python/image*.jpg
rm /home/local/svn/robobot/mqtt_python/log*.txt
# log
rm -r /home/local/svn/log/log_20*
rm /home/local/svn/log/ip_disp.out
rm /home/local/svn/log/log_*.txt
rm /home/local/svn/log/rebootinfo.txt
# these are owned by root
# rm /home/local/svn/log/rename_info.txt
# rm /home/local/svn/log/off_by*.txt
echo "all clean (except svn/log/rename_info.txt and off_by*.txt that is owned by root)"
