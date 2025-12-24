#include <stdint.h>
#include <sbi.h>
#include <private_kdefs.h>

void clock_set_next_event(void) {
  /*uint64_t time;
  asm volatile("rdtime %0" : "=r"(time));

  uint64_t next = time + TIMECLOCK;*/

  sbi_ecall(0x54494d45, 0, TIMECLOCK, 0, 0, 0, 0, 0);
}
