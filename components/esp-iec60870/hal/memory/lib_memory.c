#include <stdlib.h>
#include "esp_log.h"

void *Memory_malloc(size_t size)
{
    void *ptr = malloc(size);
    ESP_LOGI("GLOBAL_MALLOC", "ptr: %p, size: %d", ptr, size);
    return ptr;
}

void *Memory_calloc(size_t nmemb, size_t size)
{
    void *ptr = calloc(nmemb, size);
    ESP_LOGI("GLOBAL_CALLOC", "ptr: %p, nmemb: %d, size: %d", ptr, nmemb, size);
    return ptr;
}

void *Memory_realloc(void *oldptr, size_t size)
{
    void *ptr = realloc(oldptr, size);
    ESP_LOGI("GLOBAL_REALLOC", "newptr: %p, oldptr: %p, size: %d", ptr, oldptr, size);
    return ptr;
}

void Memory_free(void *ptr)
{
    ESP_LOGI("GLOBAL_FREEMEM", "ptr: %p", ptr);
    free(ptr);
}