#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y := ox820.o rps_irq.o rps_time.o pci.o ahb_mon.o timer.o 

obj-$(CONFIG_SMP) += platsmp.o hotplug.o localtimer.o

obj-$(CONFIG_OXNAS_IBW) += ibw.o

obj-$(CONFIG_GPIO_TEST)	+= gpioTest.o
