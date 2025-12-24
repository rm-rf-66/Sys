#include <mm.h>
#include <proc.h>
#include <stdlib.h>
#include <printk.h>
#include <private_kdefs.h>
#include <inttypes.h>

static struct task_struct *task[NR_TASKS]; // 线程数组，所有的线程都保存在此
static struct task_struct *idle;           // idle 线程
struct task_struct *current;               // 当前运行线程

extern void __dummy(void);
extern void __switch_to(struct task_struct *prev, struct task_struct *next);

void dummy_task(void) {
  unsigned local = 0;
  unsigned prev_cnt = 0;
  while (1) {
    if (current->counter != prev_cnt) {
      if (current->counter == 1) {
        current->counter = 0;
      }
      prev_cnt = current->counter;
      printk("[P = %u] %u\n", current->pid, ++local);
    }
  }
}

void task_init(void) {
  srand(2025);

  idle = (struct task_struct *)alloc_page();

  idle->state = TASK_RUNNING;
  idle->pid = 0;
  idle->priority = 0;
  idle->counter = 0;

  current = idle;
  task[0] = idle;

  for (int i = 1; i < NR_TASKS; ++i) {
      task[i] = (struct task_struct *)alloc_page();
      task[i]->state = TASK_RUNNING;
      task[i]->pid = i;
      task[i]->priority = rand() % (PRIORITY_MAX - PRIORITY_MIN + 1) + PRIORITY_MIN;
      task[i]->counter = 0;

      task[i]->thread.ra = (uint64_t)__dummy;
      task[i]->thread.sp = (uint64_t)task[i] + PGSIZE;

      
  }

  printk("...task_init done!\n");
}

void switch_to(struct task_struct *next) {
    if (current == next) {
        return;
    }

    /*printk("switch to [PID = %" PRIu64 "\n", 
           next->pid);*/

    struct task_struct *prev = current;
    current = next;
    __switch_to(prev, next);
}

void do_timer(void) {
  if (current->counter == 0) {
      schedule();
      return;
  }
 
  current->counter--;
  if (current->counter > 0) {
      return;
  }
  schedule();
}

void schedule(void) {
    int next_idx;
    while (1) {
        int64_t c = -1;
        next_idx = 0;
        for (int i = 1; i < NR_TASKS; i++) {
            if (task[i]->state == TASK_RUNNING && (int64_t)task[i]->counter > c) {
                c = task[i]->counter;
                next_idx = i;
            }
        }

        if (c > 0) break;

        if (c == 0) {
            for (int i = 1; i < NR_TASKS; i++) {
                task[i]->counter = task[i]->priority;
                /*printk("SET [PID = %" PRIu64 "\n", 
                       task[i]->pid);*/
            }
            continue;
        }
        
        break;
    }
    switch_to(task[next_idx]);
}
