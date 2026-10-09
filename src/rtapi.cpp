#include "rtapi.h"

#include <errno.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

static timespec g_ts;
static double g_period_ns;

void timespec_add_ns(timespec* ts, long ns) {
  ts->tv_nsec += ns;
  while (ts->tv_nsec > 1e9) {
    ts->tv_sec += 1;
    ts->tv_nsec -= 1e9;
  }
}

double timespec_compare(timespec* ts1, timespec* ts2) {
  double dts = ts1->tv_sec - ts2->tv_sec;
  double dtn = ts1->tv_nsec - ts2->tv_nsec;
  return dts * 1e9 + dtn;
}

int set_sched_prio(short prio, unsigned long period) {
  struct sched_param param;

  if ((param.sched_priority = sched_get_priority_max(SCHED_FIFO)) == -1) {
    perror("sched_get_priority_max");
    return -1;
  }
  param.sched_priority -= RT_PRIO_MAX - prio;

  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler");
    return -1;
  }
  clock_gettime(CLOCK_MONOTONIC, &g_ts);
  g_period_ns = period * 1e3;
  return 0;
}
