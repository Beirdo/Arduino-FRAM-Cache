#ifndef __memmem_h_
#define __memmem_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

// Shamelessly stolen from Android libc.
void *memmem(const void *haystack, size_t n, const void *needle, size_t m);

#ifdef __cplusplus
};
#endif

#endif
