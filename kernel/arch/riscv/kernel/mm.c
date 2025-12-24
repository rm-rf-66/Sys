#include <stdint.h>
#include <string.h>
#include <mm.h>
#include <printk.h>
#include <private_kdefs.h>

extern uint8_t _ekernel[];

static struct kfreelist {
  struct kfreelist *next;
} *kfreelist;

void *alloc_page(void) {
  struct kfreelist *r = kfreelist;
  kfreelist = r->next;
  return r;
}

void free_pages(void *addr) {
  struct kfreelist *r = (void *)PGROUNDDOWN((uintptr_t)addr);
  // memset(r, 0xfa, PGSIZE);
  r->next = kfreelist;
  kfreelist = r;
}

void mm_init(void) {
  uint8_t *s = (void *)PGROUNDUP((uintptr_t)_ekernel);
  const uint8_t *e = (void *)PHY_END;
  for (; s + PGSIZE <= e; s += PGSIZE) {
    free_pages(s);
  }

  printk("...mm_init done!\n");
}
