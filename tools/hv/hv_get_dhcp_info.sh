#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

# This example script retrieves the DHCP state of a given interface.
# In the interest of keeping the KVP daemon code free of distro specific
# information; the kvp daemon code invokes this external script to gather
# DHCP setting for the specific interface.
#
# Input: Name of the interface
#
# Output: The script prints the string "Enabled" to stdout to indicate
#	that DHCP is enabled on the interface. If DHCP is not enabled,
#	the script prints the string "Disabled" to stdout.
#
# Distributions may need to adapt or replace this script for their
# preferred network configuration system.

# Report status based on result of previous command
report() {
    if [ $? -eq 0 ]; then
	echo "Enabled"
    else
	echo "Disabled"
    fi
}

check_network_manager() {
    local conn_name

    # Check that the interface has a configured connection, and get
    # its name
    if conn_name="$(nmcli -g GENERAL.CONNECTION device show "$1" 2>/dev/null)" &&
       [ "$conn_name" ]; then
	# Check whether the connection enables DHCPv4
	test "$(nmcli -g ipv4.method connection show "$conn_name")" = auto
	report
    else
	return 1
    fi
}

check_systemd_networkd() {
    local status

    # Check that the interface is managed by networkd
    if status="$(networkctl status --json=short -- "$1" 2>/dev/null)" &&
       ! printf '%s' "$status" |
	   grep -qE '"AdministrativeState":"unmanaged"'; then
	# Check for DHCPv4 client state in the interface status
	printf '%s' "$status" | grep -q '"DHCPv4Client":'
	report
    else
	return 1
    fi
}

check_ifupdown() {
    local conf_name

    # Check that a configuration has been applied to the interface
    if command -v ifquery >/dev/null &&
       conf_name="$(ifquery --state -- "$1" | sed 's/[^=]*=//')" &&
       [ "$conf_name" ]; then
	# Check whether that configuration enables DHCPv4.
	# Unfortunately ifquery does not expose the method name, so we
	# have to grep through the configuration file(s) and make an
	# assumption about which are included.
	find /etc/network/interfaces /etc/network/interfaces.d \
	     -type f -regex '.*/[a-zA-Z0-9_-]+$' -print |
	    xargs grep -qE '^\s*iface\s+'"$conf_name"'\s+inet\s+dhcp(\s|$)'
	report
    else
	return 1
    fi
}

check_network_scripts() {
    local if_file="/etc/sysconfig/network-scripts/ifcfg-"$1

    if [ -f "$if_file" ]; then
	grep -q dhcp "$if_file"
	report
    else
	return 1
    fi
}

check_network_manager "$1" ||
check_systemd_networkd "$1" ||
check_ifupdown "$1" ||
check_network_scripts "$1" ||
report
