/*
 * @Author: JohnJeep
 * @Date: 2025-12-01 11:04:14
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-10-30 18:26:43
 * @Description:
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

int main()
{
  int rc;
  struct timespec res;

  // 精度纳秒级别，有上下文切换，操作次数多了，会有开销
  rc = clock_getres(CLOCK_MONOTONIC, &res);
  if (!rc) {
    printf("CLOCK_MONOTONIC: %ldns\n", res.tv_nsec);
  }

  // 精度为毫秒级别，没有发生上下文切换
  rc = clock_getres(CLOCK_MONOTONIC_COARSE, &res);
  if (!rc) {
    printf("CLOCK_MONOTONIC_COARSE: %ldns\n", res.tv_nsec);
  }

  return 0;
}
