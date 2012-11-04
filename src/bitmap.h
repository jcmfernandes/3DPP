#ifndef BITMAP_H
#define BITMAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if (__WORDSIZE == 32)
typedef uint32_t bitmap_cell_t;
#elif (__WORDSIZE == 64)
typedef uint64_t bitmap_cell_t;
#else
#error "bitmap only works on 32 and 64 bit word size architectures"
#endif

typedef struct {
	bitmap_cell_t *cells;
	size_t size;
} bitmap_t;

#define bitmap_required_cells(bits) ((size_t) ((bits) / (sizeof(bitmap_cell_t) * 8)))
int bitmap_create_dynamic(bitmap_t *bitmap, uint64_t bits);
int bitmap_create_static(bitmap_t *bitmap, bitmap_cell_t *cells, size_t n);
void bitmap_set(bitmap_t *bitmap, uint64_t i);
void bitmap_unset(bitmap_t *bitmap, uint64_t i);
bool bitmap_is_set(bitmap_t *bitmap, uint64_t i);
uint64_t bitmap_size(bitmap_t *bitmap);
void bitmap_free(bitmap_t *bitmap);

#endif
