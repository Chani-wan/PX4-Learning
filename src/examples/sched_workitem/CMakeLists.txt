px4_add_module(
  MODULE examples__sched_workitem
  MAIN sched_workitem
  STACK_MAIN 2000
  SRCS
    sched_workitem.cpp
  DEPENDS
    px4_work_queue
)
