#ifndef __MM_H__
#define __MM_H__

void mm_init(void);

void *alloc_page(void);
void free_pages(void *);

#endif
