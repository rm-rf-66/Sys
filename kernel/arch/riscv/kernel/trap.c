#include <stdint.h>
#include <printk.h>

void clock_set_next_event(void);
void do_timer(void);

void trap_handler(uint64_t scause, uint64_t sepc) {
  if ((scause & 0x8000000000000000) && ((scause & 0x7FFFFFFFFFFFFFFF) == 5)) {
      // printk("[S] Supervisor timer interrupt\n");
      clock_set_next_event();
      do_timer();
  }
}
