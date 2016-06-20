#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y := oxnas.o irq.o time.o pci.o

obj-$(CONFIG_OXNAS_IBW) += ibw.o

obj-$(CONFIG_GPIO_TEST) += gpioTest.o

obj-$(CONFIG_OXNAS_FRONT_LAMP_CONTROL) += leds.o

obj-$(CONFIG_WDC_FAN_OXNAS800) += wdc-fan.o

obj-$(CONFIG_WDC_LEDS_OXNAS800) += wdc-leds.o

obj-$(CONFIG_OXNAS_WD810_LEDS) += oxnas-wd810-leds.o

obj-$(CONFIG_WDC_LEDS_TRIGGER_SATA_DISK) += wdc-ledtrig-sata.o
