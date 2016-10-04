#ifndef KEXEC_SHA256_H
#define KEXEC_SHA256_H

struct kexec_sha_region {
	unsigned long start;
	unsigned long len;
};

#define SHA256_REGIONS 16

#endif /* KEXEC_SHA256_H */
