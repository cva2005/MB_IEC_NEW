/*
 *  lib_memory.h
 *
 *  Copyright 2014-2021 Michael Zillgith
 *
 *  This file is part of Platform Abstraction Layer (libpal)
 *  for libiec61850, libmms, and lib60870.
 */

#ifndef MEMORY_H_
#define MEMORY_H_

#include "hal_base.h"

#ifdef __cplusplus
extern "C" {
#endif

void* Memory_malloc(size_t size);
void *Memory_calloc(size_t nmemb, size_t size);
void *Memory_realloc(void *oldptr, size_t size);
void Memory_free(void *ptr);

#define GLOBAL_CALLOC /*Memory_*/calloc
#define GLOBAL_MALLOC /*Memory_*/malloc
#define GLOBAL_REALLOC /*Memory_*/realloc
#define GLOBAL_FREEMEM /*Memory_*/free

#ifdef __cplusplus
}
#endif

#endif /* MEMORY_H_ */
