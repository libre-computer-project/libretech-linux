OBJECT_FILES_NON_STANDARD := y

purgatory-y := purgatory.o string.o v2wrap.o purgatory-ppc64.o crtsavres.o \
		sha256.o

targets += $(purgatory-y)
PURGATORY_OBJS = $(addprefix $(obj)/,$(purgatory-y))

LDFLAGS_purgatory.ro := -pie --no-dynamic-linker -e purgatory_start \
			--no-undefined -nostartfiles -nostdlib -nodefaultlibs
targets += purgatory.ro

KBUILD_CFLAGS := $(filter-out $(CC_FLAGS_FTRACE), $(KBUILD_CFLAGS))

KBUILD_CFLAGS += -fno-zero-initialized-in-bss -fno-builtin -ffreestanding \
		 -fno-stack-protector -fno-exceptions -fpie
KBUILD_AFLAGS += -fno-exceptions -msoft-float -fpie

$(obj)/purgatory.ro: $(PURGATORY_OBJS) FORCE
		$(call if_changed,ld)

targets += kexec-purgatory.c

quiet_cmd_relocs_check = CALL    $<
      cmd_relocs_check = $(CONFIG_SHELL) $< "$(OBJDUMP)" "$(obj)/purgatory.ro"

PHONY += relocs_check
relocs_check: arch/powerpc/scripts/check-purgatory-relocs.sh $(obj)/purgatory.ro
	$(call cmd,relocs_check)

CMD_BIN2C = $(objtree)/scripts/basic/bin2c
quiet_cmd_bin2c = BIN2C   $@
      cmd_bin2c = $(CMD_BIN2C) kexec_purgatory < $< > $@

$(obj)/kexec-purgatory.c: $(obj)/purgatory.ro relocs_check FORCE
	$(call if_changed,bin2c)
	@:


obj-$(CONFIG_KEXEC_FILE)	+= kexec-purgatory.o
