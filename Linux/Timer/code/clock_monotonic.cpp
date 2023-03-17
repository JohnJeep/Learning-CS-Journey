/**
 * @author your name (you@domain.com)
 * @brief  获取操作系统的时间精度
 * @version 0.1
 * @date 2023-03-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <time.h>

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
