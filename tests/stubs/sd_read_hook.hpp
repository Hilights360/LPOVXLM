#pragma once
#include <cstdio>
size_t testSdRead(void* destination, size_t size, size_t count, FILE* file);
#define fread testSdRead
