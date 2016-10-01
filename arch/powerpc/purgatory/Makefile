OBJECT_FILES_NON_STANDARD := y

purgatory-y := purgatory.o printf.o string.o v2wrap.o hvCall.o \
		purgatory-ppc64.o console-ppc64.o crashdump_backup.o \
		crtsavres.o sha256.o

targets += $(purgatory-y)
PURGATORY_OBJS = $(addprefix $(obj)/,$(purgatory-y))

LDFLAGS_purgatory.ro := -e purgatory_start -r --no-undefined -nostartfiles \
			-nostdlib -nodefaultlibs
targets += purgatory.ro

# Default KBUILD_CFLAGS can have -pg option set when FTRACE is enabled. That
# in turn leaves some undefined symbols like __fentry__ in purgatory and not
# sure how to relocate those. Like kexec-tools, use custom flags.

KBUILD_CFLAGS := -Wall -Wstrict-prototypes -fno-strict-aliasing \
		-fno-zero-initialized-in-bss -fno-builtin -ffreestanding \
		-fno-PIC -fno-PIE -fno-stack-protector  -fno-exceptions \
		-msoft-float -MD -Os -m$(CONFIG_WORD_SIZE)
KBUILD_AFLAGS := -fno-exceptions -msoft-float -m$(CONFIG_WORD_SIZE) \
		-D__ASSEMBLY__

ifeq ($(CONFIG_CPU_LITTLE_ENDIAN),y)
KBUILD_CFLAGS += $(call cc-option,-mabi=elfv2,$(call cc-option,-mcall-aixdesc))
KBUILD_AFLAGS += $(call cc-option,-mabi=elfv2)
else
KBUILD_CFLAGS += $(call cc-option,-mcall-aixdesc)
endif

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
