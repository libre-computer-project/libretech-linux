ifeq ($(CONFIG_SECURITY_TRUSTEES_DEBUG),y)
	EXTRA_CFLAGS += -DTRUSTEES_DEBUG
endif

obj-$(CONFIG_SECURITY_TRUSTEES) := trustees.o
trustees-objs := \
	security.o fs.o \
	init.o funcs.o ../commoncap.o
