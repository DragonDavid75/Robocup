#!/bin/bash
# script to start applications after a reboot
#
# Run the app to show Raspberry's IP on the Teensy display.
mkdir -p /home/local/svn/log
cd /home/local/svn/log
# save the last reboot date
echo "================ Rebooted ================" >> rebootinfo.txt
date >> rebootinfo.txt
../robobot/ip_disp/build/ip_disp 2>/dev/null >>ip_disp.out &
# save PID for debugging
echo "ip_disp started with PID:" >> rebootinfo.txt
sleep 0.1
pgrep -l ip_disp >> rebootinfo.txt
#
# start camera server (allow camera to be detected)
sleep 0.2
cd /home/local/svn/robobot/stream_server
/usr/bin/python3 stream_server.py 2>stream_server.err >stream_server.out &
echo "python3 cam streamer started with PID:" >> /home/local/svn/log/rebootinfo.txt
sleep 0.1
pgrep -l python >> /home/local/svn/log/rebootinfo.txt
#
# start teensy_interface - allow Teensy to be detected and interface loaded
# start is postponed until date and time is updated (typically after 25sec)
sleep 0.5
cd /home/local/svn/robobot/teensy_interface/build
./teensy_interface -d -l 2>>out_err.txt >>out_console.txt &
#./teensy_interface -d -l 2>>out_err_debug.txt >>out_console_debug.txt &
echo "Teensy _interface started with PID:" >> /home/local/svn/log/rebootinfo.txt
sleep 0.1
pgrep -l teensy_i >> /home/local/svn/log/rebootinfo.txt
#
date >> /home/local/svn/log/rebootinfo.txt
exit 0

