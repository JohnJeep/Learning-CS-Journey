/*
 * @Author: JohnJeep
 * @Date: 2021-04-05 19:12:54
 * @LastEditTime: 2021-04-05 19:20:30
 * @LastEditors: Please set LastEditors
 * @Description: 使用回调函数显示两个随机数 
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* The calling function takes a single callback as a parameter. */
void PrintTwoNumbers(int (*numberSource)(void)) 
{
    int val1 = numberSource();
    int val2 = numberSource();
    printf("%d and %d\n", val1, val2);
}

/* A possible callback */
int overNineThousand(void) 
{
    return (rand()%1000) + 9001;
}

/* Another possible callback. */
int meaningOfLife(void) 
{
    return 42;
}

/* Here we call PrintTwoNumbers() with three different callbacks. */
int main(void) 
{
    time_t t;
    srand((unsigned)time(&t)); // Init seed for random function
    PrintTwoNumbers(&rand);
    PrintTwoNumbers(&overNineThousand);
    PrintTwoNumbers(&meaningOfLife);

    return 0;
}