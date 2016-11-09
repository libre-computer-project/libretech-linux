/*
 * ppc64_asm.h - common defines for PPC64 assembly parts
 *
 * Code taken from kexec-tools.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */

#include <asm/types.h>

/*
 * ABIv1 requires dot symbol while ABIv2 does not.
 */
#ifdef PPC64_ELF_ABI_v2
#define DOTSYM(a)	a
#else
#define GLUE(a, b)	a##b
#define DOTSYM(a)	GLUE(., a)
#endif
