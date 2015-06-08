/*
 * malloc.c
 *
 *  Created on: May 27, 2015
 *      Author: walmis
 */

#include "ch.h"
#include "malloc.h"

void* malloc(size_t size) {
	return chHeapAlloc(NULL, size);
}

void free(void* ptr) {
	chHeapFree(ptr);
}



