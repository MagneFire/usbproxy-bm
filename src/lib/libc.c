#include <stddef.h>
#include <stdint.h>
#include <string.h>

void *memcpy(void *d, const void *s, size_t n)
{
	uint8_t *dp = d; const uint8_t *sp = s;
	if (((uintptr_t)dp & 3) == 0 && ((uintptr_t)sp & 3) == 0)
		for (; n >= 4; n -= 4, dp += 4, sp += 4)
			*(uint32_t *)dp = *(const uint32_t *)sp;
	while (n--) *dp++ = *sp++;
	return d;
}

void *memmove(void *d, const void *s, size_t n)
{
	uint8_t *dp = d; const uint8_t *sp = s;
	if (dp < sp || dp >= sp + n)
		return memcpy(d, s, n);
	dp += n; sp += n;
	while (n--) *--dp = *--sp;
	return d;
}

void *memset(void *d, int c, size_t n)
{
	uint8_t *dp = d;
	while (n--) *dp++ = (uint8_t)c;
	return d;
}

int memcmp(const void *a, const void *b, size_t n)
{
	const uint8_t *x = a, *y = b;
	for (; n; n--, x++, y++)
		if (*x != *y) return *x - *y;
	return 0;
}

size_t strlen(const char *s) { const char *p = s; while (*p) p++; return (size_t)(p - s); }
int strcmp(const char *a, const char *b)
{
	while (*a && *a == *b) a++, b++;
	return (uint8_t)*a - (uint8_t)*b;
}
char *strchr(const char *s, int c)
{
	for (; *s; s++) if (*s == (char)c) return (char *)s;
	return c ? NULL : (char *)s;
}

unsigned long strtoul(const char *s, char **end, int base)
{
	unsigned long v = 0;
	while (*s == ' ') s++;
	if (base == 0) {
		if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) { base = 16; s += 2; }
		else base = 10;
	} else if (base == 16 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
		s += 2;
	for (;; s++) {
		int d;
		if (*s >= '0' && *s <= '9') d = *s - '0';
		else if (*s >= 'a' && *s <= 'f') d = *s - 'a' + 10;
		else if (*s >= 'A' && *s <= 'F') d = *s - 'A' + 10;
		else break;
		if (d >= base) break;
		v = v * base + (unsigned long)d;
	}
	if (end) *end = (char *)s;
	return v;
}
