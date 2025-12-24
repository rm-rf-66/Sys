#include <stdio.h>
#include <printk.h>
#include <sbi.h>

static int printk_sbi_write(FILE *restrict fp, const void *restrict buf, size_t len) {
  (void)fp;

  
  struct sbiret ret = sbi_ecall(0x4442434e, 0, len, (uint64_t)buf, 0, 0, 0, 0);
  return ret.value;
}

void printk(const char *fmt, ...) {
  FILE printk_out = {
      .write = printk_sbi_write,
  };

  va_list ap;
  va_start(ap, fmt);
  vfprintf(&printk_out, fmt, ap);
  va_end(ap);
}
