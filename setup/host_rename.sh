#!/bin/sh
#
# /etc/init.d/host_rename.sh
# Subsystem file for "Robobot" host rename function
#
# chkconfig: 2345 95 05
# NOTE, these initial comments are needed!
# description: Robobot rename service
#
# processname: none
# config: /home/local/svn/robobot/setup/none
# pidfile: /var/run/Robobot.pid

# source function library
#. /etc/rc.d/init.d/functions

# pull in sysconfig settings
# [ -f /etc/sysconfig/mySystem ] && . /etc/sysconfig/mySystem
#
RETVAL=0
prog="teensy_interfac"
/home/local/svn/robobot/setup/rename_host.bash &
#
exit $RETVAL
