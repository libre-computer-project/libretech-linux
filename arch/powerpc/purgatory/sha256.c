#include "../boot/string.h"

/* Avoid including x86's boot/string.h in sha256.c. */
#define BOOT_STRING_H

#include "../../x86/purgatory/sha256.c"
