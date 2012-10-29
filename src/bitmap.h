#ifndef BITMAP_H
#define BITMAP_H

#include <stdbool.h>
#include <stdint.h>

#if (__WORDSIZE == 32)
#define __BITMAP_DATATYPE uint32_t
#elif (__WORDSIZE == 64)
#define __BITMAP_DATATYPE uint32_t // TODO: change to uint64_t
#else
#error "bitmap only works on 32 and 64 bit word size architectures"
#endif

typedef struct {
	__BITMAP_DATATYPE * array;
	uint64_t size;
} * bitmap_t;

bitmap_t bitmap_create(uint64_t n);
void bitmap_set(bitmap_t b, uint64_t i);
void bitmap_unset(bitmap_t b, uint64_t i);
bool bitmap_is_set(bitmap_t b, uint64_t i);
void bitmap_destroy(bitmap_t b);

#endif
