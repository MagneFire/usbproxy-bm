/* Minimal vsnprintf: %c %s %d %i %u %x %X %p %%, flags '-' '0', width,
 * .precision (for %s), 'l' accepted and ignored (32-bit). */
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "fw.h"

struct out { char *p; size_t n, len; };

static void put(struct out *o, char c)
{
	if (o->len + 1 < o->n) o->p[o->len] = c;
	o->len++;
}

static void pad(struct out *o, int n, char c) { while (n-- > 0) put(o, c); }

static void utoa_base(char *buf, unsigned long v, unsigned base, int upper)
{
	char tmp[16]; int i = 0;
	const char *dig = upper ? "0123456789ABCDEF" : "0123456789abcdef";
	do { tmp[i++] = dig[v % base]; v /= base; } while (v);
	while (i) *buf++ = tmp[--i];
	*buf = 0;
}

int vsnprintf(char *buf, size_t n, const char *fmt, va_list ap)
{
	struct out o = { buf, n, 0 };
	for (; *fmt; fmt++) {
		if (*fmt != '%') { put(&o, *fmt); continue; }
		fmt++;
		int left = 0, zero = 0, width = 0, prec = -1;
		for (;; fmt++) {
			if (*fmt == '-') left = 1;
			else if (*fmt == '0') zero = 1;
			else break;
		}
		while (*fmt >= '0' && *fmt <= '9') width = width * 10 + (*fmt++ - '0');
		if (*fmt == '.') { fmt++; prec = 0; while (*fmt >= '0' && *fmt <= '9') prec = prec * 10 + (*fmt++ - '0'); }
		while (*fmt == 'l' || *fmt == 'z') fmt++;
		char num[24]; const char *s = num; int slen; int neg = 0;
		switch (*fmt) {
		case 'c': num[0] = (char)va_arg(ap, int); num[1] = 0; break;
		case 's': s = va_arg(ap, const char *); if (!s) s = "(null)"; break;
		case 'd': case 'i': {
			long v = va_arg(ap, int);
			if (v < 0) { neg = 1; v = -v; }
			utoa_base(num, (unsigned long)v, 10, 0); break; }
		case 'u': utoa_base(num, va_arg(ap, unsigned), 10, 0); break;
		case 'x': utoa_base(num, va_arg(ap, unsigned), 16, 0); break;
		case 'X': utoa_base(num, va_arg(ap, unsigned), 16, 1); break;
		case 'p': utoa_base(num, (unsigned long)(uintptr_t)va_arg(ap, void *), 16, 0); break;
		case '%': put(&o, '%'); continue;
		default: put(&o, '%'); put(&o, *fmt); continue;
		}
		slen = (int)strlen(s);
		if (prec >= 0 && *fmt == 's' && slen > prec) slen = prec;
		int total = slen + neg;
		if (!left) pad(&o, width - total, zero && *fmt != 's' ? '0' : ' ');
		if (neg) put(&o, '-');
		for (int i = 0; i < slen; i++) put(&o, s[i]);
		if (left) pad(&o, width - total, ' ');
	}
	if (n) buf[o.len < n ? o.len : n - 1] = 0;
	return (int)o.len;
}

int snprintf(char *buf, size_t n, const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	int r = vsnprintf(buf, n, fmt, ap);
	va_end(ap);
	return r;
}
