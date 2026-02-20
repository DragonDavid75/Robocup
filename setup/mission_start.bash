#!/bin/bash
# echo -e "\nStarting mission\n"
cd /home/local/svn/robobot/mqtt_python
/usr/bin/python3 mqtt-client.py -n >>log_out.txt 2>>log_err.txt &
# echo "mission ended"
exit 0
