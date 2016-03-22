#!/bin/bash
#
# Analyze a given results directory for waketorture diagnostics.
#
# Usage: kvm-recheck-wake resdir
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, you can access it online at
# http://www.gnu.org/licenses/gpl-2.0.html.
#
# Copyright (C) IBM Corporation, 2016
#
# Authors: Paul E. McKenney <paulmck@linux.vnet.ibm.com>

i="$1"
if test -d $i
then
	:
else
	echo Unreadable results directory: $i
	exit 1
fi
PATH=`pwd`/tools/testing/selftests/rcutorture/bin:$PATH; export PATH
. tools/testing/selftests/rcutorture/bin/functions.sh

configfile=`echo $i | sed -e 's/^.*\///'`

sed -e 's/^\[[^]]*]//' < $i/console.log | grep -e -torture |
	grep "Tardy kthreads" | tail -1
