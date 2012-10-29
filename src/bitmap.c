#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "bitmap.h"

static const __BITMAP_DATATYPE bitmap_mask_set[] = {
		0x80000000, 0x40000000, 0x20000000, 0x10000000,
		0x8000000, 0x4000000, 0x2000000, 0x1000000,
		0x800000, 0x400000, 0x200000, 0x100000,
		0x80000, 0x40000, 0x20000, 0x10000,
		0x8000, 0x4000, 0x2000, 0x1000,
		0x800, 0x400, 0x200, 0x100,
		0x80, 0x40, 0x20, 0x10,
		0x8, 0x4, 0x2, 0x1
};

static const __BITMAP_DATATYPE bitmap_mask_unset[] = {
		0x7FFFFFFF, 0xBFFFFFFF, 0xDFFFFFFF, 0xEFFFFFFF,
		0xF7FFFFFF, 0xFBFFFFFF, 0xFDFFFFFF, 0xFEFFFFFF,
		0xFF7FFFFF, 0xFFBFFFFF, 0xFFDFFFFF, 0xFFEFFFFF,
		0xFFF7FFFF, 0xFFFBFFFF, 0xFFFDFFFF, 0xFFFEFFFF,
		0xFFFF7FFF, 0xFFFFBFFF, 0xFFFFDFFF, 0xFFFFEFFF,
		0xFFFFF7FF, 0xFFFFFBFF, 0xFFFFFDFF, 0xFFFFFEFF,
		0xFFFFFF7F, 0xFFFFFFBF, 0xFFFFFFDF, 0xFFFFFFEF,
		0xFFFFFFF7, 0xFFFFFFFB, 0xFFFFFFFD, 0xFFFFFFFE,
};

bitmap_t bitmap_create(uint64_t n)
{
	bitmap_t b = malloc(sizeof(bitmap_t));
    b->array = calloc((n / sizeof(__BITMAP_DATATYPE)) + 1, sizeof(__BITMAP_DATATYPE));
    b->size = n;
    return b;
}

void bitmap_set(bitmap_t b, uint64_t i)
{
	b->array[i / sizeof(__BITMAP_DATATYPE)] |= bitmap_mask_set[i & (sizeof(__BITMAP_DATATYPE) - 1)];
}

void bitmap_unset(bitmap_t b, uint64_t i)
{
	b->array[i / sizeof(__BITMAP_DATATYPE)] &= bitmap_mask_unset[i & (sizeof(__BITMAP_DATATYPE) - 1)];
}

bool bitmap_is_set(bitmap_t b, uint64_t i)
{
	return b->array[i / sizeof(__BITMAP_DATATYPE)] & bitmap_mask_set[i & (sizeof(__BITMAP_DATATYPE) - 1)] ? true : false;
}

void bitmap_destroy(bitmap_t b)
{
	free(b->array);
    free(b);
}

