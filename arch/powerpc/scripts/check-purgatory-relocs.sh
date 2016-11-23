#!/bin/sh

# Copyright © 2016 IBM Corporation

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version
# 2 of the License, or (at your option) any later version.

# This script checks the relocations of a purgatory executable for
# relocations that are not handled by the kernel.

# Based on relocs_check.sh.
# Copyright © 2015 IBM Corporation

if [ $# -lt 2 ]; then
	echo "$0 [path to objdump] [path to purgatory.ro]" 1>&2
	exit 1
fi

# Have Kbuild supply the path to objdump so we handle cross compilation.
objdump="$1"
purgatory="$2"

bad_relocs=$(
"$objdump" -R "$purgatory" |
	# Only look at relocation lines.
	grep -E '\<R_' |
	# These relocations are okay
	# On PPC64:
	#	R_PPC64_ADDR16_LO, R_PPC64_ADDR16_HI,
	#	R_PPC64_ADDR16_HIGHER, R_PPC64_ADDR16_HIGHEST,
	#	R_PPC64_RELATIVE
	grep -F -w -v 'R_PPC64_ADDR16_LO
R_PPC64_ADDR16_HI
R_PPC64_ADDR16_HIGHER
R_PPC64_ADDR16_HIGHEST
R_PPC64_RELATIVE'
)

if [ -z "$bad_relocs" ]; then
	exit 0
fi

num_bad=$(echo "$bad_relocs" | wc -l)
echo "WARNING: $num_bad bad relocations in $2"
echo "$bad_relocs"
