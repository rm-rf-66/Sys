#ifndef __CONV_H__
#define __CONV_H__
typedef unsigned long long int uint64_t;
typedef uint64_t size_t;

void conv_compute(const uint64_t* data_array, size_t data_len, const uint64_t* kernel_array, size_t kernel_len, uint64_t* dest);
void mul_compute(const uint64_t* data_array, size_t data_len, const uint64_t* kernel_array, size_t kernel_len, uint64_t* dest);
uint64_t get_time(void);

#endif