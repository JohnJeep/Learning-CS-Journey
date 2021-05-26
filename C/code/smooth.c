/*
 * @Author: JohnJeep
 * @Date: 2020-04-02 15:47:47
 * @LastEditTime: 2021-05-26 22:32:41
 * @LastEditors: Please set LastEditors
 * @Description: 滑动平均滤波算法
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

#define N 6
int value_buff[N];           //N相当于选定一个窗口大小，对窗口数据做平均！
int i=0;

int get_data()
{
    int data = 0;
    printf("please input data:");

    for (int j = 0; i < 6; j++) {
        scanf("%d\n",&data);

    }
    return data;
}

int filter()
{
    int count;
    int sum=0;

    value_buff[i++] = get_data();
    if(i==N)
    i=0;                        //当数据大于数组长度，替换数据组的一个数据  相当于环形队列更新，先进先出！
    for(count=0;count<N;count++)
        sum+=value_buff[count];
    return (int)(sum/N);
}

int main()
{
    // get_data();
    filter();

    return 0;
}