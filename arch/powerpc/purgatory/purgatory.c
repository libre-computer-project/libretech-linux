/*
 * Code taken from kexec-tools.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "purgatory.h"
#include "sha256.h"
#include "../boot/string.h"
#include "kexec-sha256.h"

struct kexec_sha_region sha_regions[SHA256_REGIONS] __section(".data");
u8 sha256_digest[SHA256_DIGEST_SIZE] __section(".data");

int verify_sha256_digest(void)
{
	struct kexec_sha_region *ptr, *end;
	u8 digest[SHA256_DIGEST_SIZE];
	size_t i;
	struct sha256_state sctx;

	sha256_init(&sctx);
	end = &sha_regions[sizeof(sha_regions)/sizeof(sha_regions[0])];
	for (ptr = sha_regions; ptr < end; ptr++)
		sha256_update(&sctx, (uint8_t *)(ptr->start), ptr->len);
	sha256_final(&sctx, digest);

	if (memcmp(digest, sha256_digest, sizeof(digest)) != 0) {
		printf("sha256 digests do not match :(\n");
		printf("       digest: ");
		for (i = 0; i < sizeof(digest); i++)
			printf("%hhx ", digest[i]);
		printf("\n");

		printf("sha256_digest: ");
		for (i = 0; i < sizeof(sha256_digest); i++)
			printf("%hhx ", sha256_digest[i]);

		printf("\n");
		return 1;
	}
	return 0;
}

void purgatory(void)
{
	printf("I'm in purgatory\n");
	setup_arch();
	if (verify_sha256_digest()) {
		/* loop forever */
		for (;;)
			;
	}
	post_verification_setup_arch();
}
