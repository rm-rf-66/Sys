#include "conv.h"

typedef unsigned long long int size_t;
uint64_t* CONV_BASE = (uint64_t*)0x10001000L;
const size_t CONV_KERNEL_OFFSET = 0;
const size_t CONV_DATA_OFFSET = 1;
const size_t CONV_RESULT_LO_OFFSET = 0;
const size_t CONV_RESULT_HI_OFFSET = 1;
const size_t CONV_STATE_OFFSET = 2;
const unsigned char READY_MASK = 0b01;
const size_t CONV_ELEMENT_LEN = 4;

uint64_t* MISC_BASE = (uint64_t*)0x10002000L;
const size_t MISC_TIME_OFFSET = 0;

uint64_t get_time(void){
    return MISC_BASE[MISC_TIME_OFFSET];
}

void wait_conv_ready() {
    while ((CONV_BASE[CONV_STATE_OFFSET] & READY_MASK) == 0) {
    }
}

void simple_64bit_multiply(uint64_t a, uint64_t b, uint64_t* high, uint64_t* low) {
    *low = 0;
    *high = 0;
    
    for (int i = 0; i < 64; i++) {
        if (b & 1) {
            uint64_t temp_low = *low;
            *low += (a << i);
            *high += (a >> (64 - i));
            if (*low < temp_low) (*high)++; 
        }
        b >>= 1;
    }
}

void conv_compute(const uint64_t* data_array, size_t data_len, const uint64_t* kernel_array, size_t kernel_len, uint64_t* dest){
    for (size_t i = 0; i < kernel_len; i++) {
        CONV_BASE[CONV_KERNEL_OFFSET] = kernel_array[i];
    }
    
    size_t len = kernel_len - 1;

    for (size_t i = 0; i < len; i++) {
        CONV_BASE[CONV_DATA_OFFSET] = 0;
    }

    for (size_t i = 0; i < data_len; i++) {
        CONV_BASE[CONV_DATA_OFFSET] = data_array[i];

        while ((CONV_BASE[CONV_STATE_OFFSET] & READY_MASK) == 0);

        dest[i * 2] = CONV_BASE[CONV_RESULT_HI_OFFSET];       
        dest[i * 2 + 1] = CONV_BASE[CONV_RESULT_LO_OFFSET]; 
    }

    for (size_t i = 0; i < len; i++) {
        CONV_BASE[CONV_DATA_OFFSET] = 0;

        while ((CONV_BASE[CONV_STATE_OFFSET] & READY_MASK) == 0);

        dest[(i + data_len) * 2] = CONV_BASE[CONV_RESULT_HI_OFFSET];
        dest[((i + data_len) * 2) + 1] = CONV_BASE[CONV_RESULT_LO_OFFSET];
    }
}

void mul_compute(const uint64_t* data_array, size_t data_len, const uint64_t* kernel_array, size_t kernel_len, uint64_t* dest){
    for (size_t i = 0; i < data_len + (kernel_len - 1); i++) {
        uint64_t acc_high = 0;  
        uint64_t acc_low = 0;   

        for (size_t j = 0; j < kernel_len; j++) {
            uint64_t data;
            if (i + j >= kernel_len - 1 && i + j < data_len + kernel_len - 1) {
                data = data_array[i + j - (kernel_len - 1)];
            } else {
                data = 0;
            }

            uint64_t kernel = kernel_array[j];

            uint64_t mul_high, mul_low;
            simple_64bit_multiply(data, kernel, &mul_high, &mul_low);

            uint64_t old_low = acc_low;
            acc_low += mul_low;
            acc_high += mul_high;
            
            if (acc_low < old_low) {
                acc_high++;
            }
        }
        
        dest[i << 1] = acc_high;        
        dest[(i << 1) + 1] = acc_low;   
    }
}
