#ifndef __PROC_H__
#define __PROC_H__

#include <stdint.h>

#define TASK_RUNNING 0 // 为了简化实验，所有的线程都只有一种状态

// 可自行修改的宏定义
#define NR_TASKS (1 + 4) // idle 线程 + 用户线程
#define PRIORITY_MIN 1
#define PRIORITY_MAX 10

// 线程上下文
struct thread_struct {
  uint64_t ra;
  uint64_t sp;
  uint64_t s[12];
};

// 线程数据
struct task_struct {
  uint64_t pid;      // 线程 ID
  uint64_t state;    // 状态
  uint64_t priority; // 优先级
  uint64_t counter;  // 剩余时间

  struct thread_struct thread; // 线程上下文
};


void task_init(void);
void do_timer(void);
void schedule(void);
void switch_to(struct task_struct *next);
void dummy_task(void);

#endif
