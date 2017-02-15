CFLAGS += -Wall -O2 -lcap -I../../../include/uapi -I../../../lib

TEST_GEN_PROGS = test_verifier test_tag test_maps test_lru_map test_lpm_map

TEST_PROGS := test_kmod.sh

$(test_objs): ../../../lib/bpf/bpf.o

include ../lib.mk
