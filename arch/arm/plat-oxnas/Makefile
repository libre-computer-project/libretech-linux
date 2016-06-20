#
# Makefile for the linux kernel.
#

obj-y :=
obj-m :=
obj-n :=
obj-  :=

obj-y += dma.o
obj-y += leon.o
obj-y += samba_reserve.o

obj-$(CONFIG_TACHO_THERM_AND_FAN) 	   += thermAndFan.o
obj-$(CONFIG_OXNAS_AHB_MONITOR_MODULE)	   += oxnas-ahb-monitor.o
obj-$(CONFIG_CRYPTO_OXAESLRW)		   += cipher.o
obj-$(CONFIG_I2S)			   += i2s.o
obj-$(CONFIG_DPE_TEST)			   += dpe_test.o
obj-$(CONFIG_OXNAS_USB_TEST_MODES)	   += usb-test-mode.o
obj-$(CONFIG_LEON_POWER_BUTTON_MONITOR)	   += power_button.o
obj-$(CONFIG_USER_RECOVERY_BUTTON_MONITOR) += user_recovery_button.o

# OX800
obj-$(CONFIG_OXNAS_VERSION_0X800) +=

# OX810
obj-$(CONFIG_OXNAS_VERSION_0X810) +=

# OX820
obj-$(CONFIG_ARCH_0X820) +=