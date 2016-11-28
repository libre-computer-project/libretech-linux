#ifndef PURGATORY_H
#define PURGATORY_H

#include <linux/types.h>

int memcmp(const void *cs, const void *ct, size_t count);
void setup_arch(void);
void post_verification_setup_arch(void);

#endif /* PURGATORY_H */
