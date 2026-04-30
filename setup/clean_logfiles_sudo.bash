#!/bin/bash
# clean logfiles before flash image
# these are owned by root
rm /home/local/svn/log/rename_info.txt
rm /home/local/svn/log/off_by*.txt
echo "all root-logfiles clean"
