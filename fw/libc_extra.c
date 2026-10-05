/* Functions missing from LiteX's minimal picolibc build. */
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int memcmp(const void* a, const void* b, size_t n) {
    const unsigned char *p = a, *q = b;
    for (; n; --n, ++p, ++q)
        if (*p != *q)
            return *p - *q;
    return 0;
}

char* strrchr(const char* s, int c) {
    const char* r = NULL;
    for (;; ++s) {
        if (*s == (char)c)
            r = s;
        if (!*s)
            return (char*)r;
    }
}

int atoi(const char* s) {
    while (*s == ' ')
        ++s;
    const int neg = *s == '-';
    if (*s == '-' || *s == '+')
        ++s;
    const long v = (long)strtoul(s, NULL, 10);
    return (int)(neg ? -v : v);
}

char* strtok(char* s, const char* delim) {
    static char* save;
    if (!s)
        s = save;
    if (!s)
        return NULL;
    while (*s && strchr(delim, *s))
        ++s;
    if (!*s) {
        save = NULL;
        return NULL;
    }
    char* t = s;
    while (*s && !strchr(delim, *s))
        ++s;
    if (*s)
        *s++ = 0;
    save = s;
    return t;
}

/* Minimal snprintf: %s %c %d %i %u %x %ld %lu %lx %%, optional width/zero-pad. */
int vsnprintf(char* out, size_t size, const char* fmt, va_list ap) {
    size_t n = 0;
#define PUT(ch) do { if (n + 1 < size) out[n] = (ch); ++n; } while (0)
    for (; *fmt; ++fmt) {
        if (*fmt != '%') { PUT(*fmt); continue; }
        ++fmt;
        int zero = 0, width = 0, lng = 0;
        if (*fmt == '0') { zero = 1; ++fmt; }
        while (*fmt >= '0' && *fmt <= '9') width = width * 10 + (*fmt++ - '0');
        while (*fmt == 'l') { lng = 1; ++fmt; }
        char buf[24];
        int len = 0, neg = 0;
        unsigned long u = 0;
        switch (*fmt) {
        case 's': {
            const char* s = va_arg(ap, const char*);
            if (!s) s = "(null)";
            int l = (int)strlen(s);
            for (int i = l; i < width; ++i) PUT(' ');
            while (*s) PUT(*s++);
            continue;
        }
        case 'c': PUT((char)va_arg(ap, int)); continue;
        case '%': PUT('%'); continue;
        case 'd': case 'i': {
            long v = lng ? va_arg(ap, long) : va_arg(ap, int);
            if (v < 0) { neg = 1; u = (unsigned long)(-v); } else u = (unsigned long)v;
            break;
        }
        case 'u': case 'x': case 'X':
            u = lng ? va_arg(ap, unsigned long) : va_arg(ap, unsigned int);
            break;
        default: PUT('%'); PUT(*fmt); continue;
        }
        const unsigned base = (*fmt == 'x' || *fmt == 'X') ? 16 : 10;
        do { buf[len++] = "0123456789abcdef"[u % base]; u /= base; } while (u);
        if (neg) width--;
        if (neg && zero) PUT('-');
        for (int i = len; i < width; ++i) PUT(zero ? '0' : ' ');
        if (neg && !zero) PUT('-');
        while (len) PUT(buf[--len]);
    }
    if (size) out[n < size ? n : size - 1] = 0;
#undef PUT
    return (int)n;
}

int snprintf(char* out, size_t size, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const int r = vsnprintf(out, size, fmt, ap);
    va_end(ap);
    return r;
}
