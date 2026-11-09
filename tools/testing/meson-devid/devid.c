// SPDX-License-Identifier: GPL-2.0-only
/*
 * devid -- exercise /dev/devid-enroll and /dev/devid from userspace.
 *
 *   devid ident                print board, UID and E
 *   devid burn <blob-file>     Identify, then Burn the blob, on one open fd
 *   devid answer <nonce-hex>   print result, serial and MAC
 *   devid busy                 open the enroll node twice: the second open
 *                              must fail EBUSY
 *   devid enroll <hex-file> <go-file>
 *                              live enrollment, one open fd throughout:
 *                              Identify and print E, then wait (up to 15 min)
 *                              for <hex-file> to hold the blob as hex (a
 *                              server mints it for THAT E), print the parsed
 *                              blob back, then wait for <go-file>: "1" burns,
 *                              anything else gives up without burning
 *   devid asuser <uid> <cmd> [args]
 *                              drop to uid:gid <uid> (no supplementary
 *                              groups), then run <cmd> as above
 *
 * Every line starts `DEVID ` so a harness can score it.
 */

#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <linux/meson-devid.h>

/* Read a hex file of exactly n bytes into out; 0 when complete. */
static int read_hex(const char *path, unsigned char *out, size_t n)
{
	FILE *f = fopen(path, "r");
	size_t got = 0;
	int c, hi = -1;

	if (!f)
		return -1;
	while (got < n && (c = fgetc(f)) != EOF) {
		int v;

		if (c >= '0' && c <= '9')
			v = c - '0';
		else if (c >= 'a' && c <= 'f')
			v = c - 'a' + 10;
		else if (c >= 'A' && c <= 'F')
			v = c - 'A' + 10;
		else
			continue;
		if (hi < 0) {
			hi = v;
		} else {
			out[got++] = (unsigned char)(hi << 4 | v);
			hi = -1;
		}
	}
	fclose(f);
	return got == n ? 0 : -1;
}

static int wait_for(const char *path, int seconds)
{
	struct stat st;

	while (seconds-- > 0) {
		if (stat(path, &st) == 0 && st.st_size > 0)
			return 0;
		sleep(1);
	}
	return -1;
}

static void hex(const char *label, const unsigned char *p, size_t n)
{
	size_t i;

	printf(" %s=", label);
	for (i = 0; i < n; i++)
		printf("%02x", p[i]);
}

static int ident(int fd)
{
	struct meson_devid_ident id;

	if (ioctl(fd, MESON_DEVID_IOC_IDENTIFY, &id) != 0) {
		printf("DEVID ident errno=%d\n", errno);
		return 1;
	}
	printf("DEVID ident magic=%.4s board=0x%x", id.magic, id.board_id);
	hex("uid", id.uid, id.uid_len);
	hex("E", id.E, sizeof(id.E));
	printf("\n");
	return 0;
}

static int run(int argc, char **argv);

int main(int argc, char **argv)
{
	if (argc > 3 && !strcmp(argv[1], "asuser")) {
		uid_t id = (uid_t)strtoul(argv[2], NULL, 0);

		if (setgroups(0, NULL) || setgid(id) || setuid(id)) {
			printf("DEVID asuser errno=%d\n", errno);
			return 1;
		}
		printf("DEVID asuser uid=%u gid=%u\n", getuid(), getgid());
		return run(argc - 2, argv + 2);
	}
	return run(argc, argv);
}

static int run(int argc, char **argv)
{
	int fd, fd2;

	if (argc < 2)
		return 2;
	if (!strcmp(argv[1], "enroll") && argc > 3) {
		struct meson_devid_burn b;
		char go[4] = "";
		FILE *g;
		size_t i;

		fd = open("/dev/devid-enroll", O_RDWR);
		if (fd < 0) {
			printf("DEVID open errno=%d\n", errno);
			return 1;
		}
		if (ident(fd))
			return 1;
		fflush(stdout);
		memset(&b, 0, sizeof(b));
		if (wait_for(argv[2], 900) || read_hex(argv[2], b.blob, sizeof(b.blob))) {
			printf("DEVID enroll no-blob\n");
			return 1;
		}
		printf("DEVID enroll blob=");
		for (i = 0; i < sizeof(b.blob); i++)
			printf("%02x", b.blob[i]);
		printf("\n");
		fflush(stdout);
		if (wait_for(argv[3], 900)) {
			printf("DEVID enroll no-go\n");
			return 1;
		}
		g = fopen(argv[3], "r");
		if (g && !fgets(go, sizeof(go), g))
			go[0] = '\0';
		if (g)
			fclose(g);
		if (go[0] != '1') {
			printf("DEVID enroll abandoned\n");
			return 1;
		}
		if (ioctl(fd, MESON_DEVID_IOC_BURN, &b) != 0) {
			printf("DEVID burn errno=%d\n", errno);
			return 1;
		}
		printf("DEVID burn result=%d\n", b.result);
		close(fd);
		return 0;
	}
	if (!strcmp(argv[1], "ident") || !strcmp(argv[1], "burn")) {
		fd = open("/dev/devid-enroll", O_RDWR);
		if (fd < 0) {
			printf("DEVID open errno=%d\n", errno);
			return 1;
		}
		if (ident(fd))
			return 1;
		if (!strcmp(argv[1], "burn")) {
			struct meson_devid_burn b;
			FILE *f = argc > 2 ? fopen(argv[2], "rb") : NULL;

			memset(&b, 0, sizeof(b));
			if (!f || fread(b.blob, 1, sizeof(b.blob), f) != sizeof(b.blob))
				return 2;
			fclose(f);
			if (ioctl(fd, MESON_DEVID_IOC_BURN, &b) != 0) {
				printf("DEVID burn errno=%d\n", errno);
				return 1;
			}
			printf("DEVID burn result=%d\n", b.result);
		}
		close(fd);
		return 0;
	}
	if (!strcmp(argv[1], "answer") && argc > 2) {
		struct meson_devid_answer a;
		size_t i, n = strlen(argv[2]) / 2;

		memset(&a, 0, sizeof(a));
		if (n == 0 || n > sizeof(a.nonce))
			return 2;
		for (i = 0; i < n; i++) {
			unsigned int v;

			if (sscanf(argv[2] + 2 * i, "%2x", &v) != 1)
				return 2;
			a.nonce[i] = (unsigned char)v;
		}
		a.nonce_len = n;
		fd = open("/dev/devid", O_RDWR);
		if (fd < 0 || ioctl(fd, MESON_DEVID_IOC_ANSWER, &a) != 0) {
			printf("DEVID answer errno=%d\n", errno);
			return 1;
		}
		printf("DEVID answer result=%d serial=%u", a.result, a.serial);
		hex("mac", a.mac, sizeof(a.mac));
		printf("\n");
		close(fd);
		return 0;
	}
	if (!strcmp(argv[1], "busy")) {
		fd = open("/dev/devid-enroll", O_RDWR);
		fd2 = open("/dev/devid-enroll", O_RDWR);
		printf("DEVID busy first=%s second_errno=%d\n",
		       fd >= 0 ? "ok" : "fail", fd2 < 0 ? errno : 0);
		return 0;
	}
	return 2;
}
