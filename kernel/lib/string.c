#include <string.h>

void *memset(void *restrict dst, int c, size_t n) {
    unsigned char *p = dst;
    while (n--) {
        *p++ = (unsigned char)c;
    }
    return dst;
}

size_t strnlen(const char *restrict s, size_t maxlen) {
    size_t len = 0;
    while (len < maxlen && s[len] != '\0') {
        len++;
    }
    return len;
}
