OBJECT_FILES_NON_STANDARD := y

purgatory-y := purgatory.o printf.o string.o v2wrap.o hvCall.o \
		purgatory-ppc64.o console-ppc64.o  crtsavres.o sha256.o

targets += $(purgatory-y)
PURGATORY_OBJS = $(addprefix $(obj)/,$(purgatory-y))

LDFLAGS_purgatory.ro := -e purgatory_start -r --no-undefined -nostartfiles \
			-nostdlib -nodefaultlibs
targets += purgatory.ro

KBUILD_CFLAGS := $(filter-out $(CC_FLAGS_FTRACE), $(KBUILD_CFLAGS))

KBUILD_CFLAGS += -fno-zero-initialized-in-bss -fno-builtin -ffreestanding \
		 -fno-stack-protector -fno-exceptions -fpie
KBUILD_AFLAGS += -fno-exceptions -msoft-float -fpie

$(obj)/purgatory.ro: $(PURGATORY_OBJS) FORCE
		$(call if_changed,ld)

targets += kexec-purgatory.c

CMD_BIN2C = $(objtree)/scripts/basic/bin2c
quiet_cmd_bin2c = BIN2C   $@
      cmd_bin2c = $(CMD_BIN2C) kexec_purgatory < $< > $@

$(obj)/kexec-purgatory.c: $(obj)/purgatory.ro FORCE
	$(call if_changed,bin2c)
	@:


obj-$(CONFIG_KEXEC_FILE)	+= kexec-purgatory.o
