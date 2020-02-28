/****************************************************************************
 * sched/pthread/pthread_setaffinity.c
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>

#include "pthread/pthread.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setaffinity
 *
 * Description:
 *   The pthread_setaffinity_np() function sets the CPU affinity mask of
 *   the thread thread to the CPU set pointed to by cpuset.  If the call
 *   is successful, and the thread is not currently running on one of the
 *   CPUs in cpuset, then it is migrated to one of those CPUs.

 *   The argument cpusetsize is the length (in bytes) of the buffer
 *   pointed to by cpuset.  Typically, this argument would be specified as
 *   sizeof(cpu_set_t).
 *
 * Input Parameters:
 *   thread     - The ID of thread whose affinity set will be modified.
 *   cpusetsize - Size of cpuset.  MUST be sizeofcpu_set_t().
 *   cpuset     - Provides the new affinity set for the thread.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an errno value is returned indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int pthread_setaffinity_np(pthread_t thread, size_t cpusetsize,
                           FAR const cpu_set_t *cpuset)
{
  int ret;

  sinfo("thread ID=%d cpusetsize=%d cpuset=%p\n",
        (int)thread, (int)cpusetsize, cpusetsize);

  DEBUGASSERT(thread > 0 && cpusetsize == sizeof(cpu_set_t) &&
              cpuset != NULL);

  /* Let nxsched_setaffinity do all of the work, adjusting the return value */

  ret = nxsched_setaffinity((pid_t)thread, cpusetsize, cpuset);
  return ret < 0 ? -ret : OK;
}

#endif /* CONFIG_SMP */
