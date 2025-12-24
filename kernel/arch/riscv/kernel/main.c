#include <printk.h>

_Noreturn void start_kernel(void) {
  printk("2025 ZJU Computer System II\n");

  // 等待第一次时钟中断
  while (1)
    ;
}
