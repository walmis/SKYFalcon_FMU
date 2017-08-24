/*
 * malloc.c
 *
 *  Created on: May 27, 2015
 *      Author: walmis
 */

#include "ch.h"
#include "malloc.h"
#include "string.h"

void* malloc(size_t size) {
	void* alloc = chHeapAlloc(NULL, size);
	if(!alloc) return alloc;
	memset(alloc, 0, size);
	return alloc;
}

void free(void* ptr) {
	chHeapFree(ptr);
}



