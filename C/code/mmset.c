/*
 * @Author: JohnJeep
 * @Date: 2019-09-02 09:04:14
 * @LastEditTime: 2021-05-26 22:24:23
 * @LastEditors: Please set LastEditors
 * @Description: memset函数的用法实现
 */

#include "stdio.h"
#include "memory.h"


int main() 
{
    printf("hello C!\n");

    char buf[] = {"set memset !"};
    printf("use memset before: %s \n", buf);

    memset(buf, '*', sizeof(buf));
    printf("use memset after: %s \n", buf);
    printf("%d", sizeof(buf));

    // 数组初始化
    int arr[15];
    printf("数组使用memset前: %d \n", arr[1]);  // 数组不是使用下标,系统会默认打印下标为0的地址
    printf("数组使用memset前的地址: %x \n", &arr[1]);
    memset(arr, 0, sizeof(int)*15); 
    printf("数组使用memset后：%d\n", arr[1]);
    printf("数组使用memset后的地址：%x\n", &arr[1]);

    int a[10] = {0};
    printf("数组a的初值：%d \n", a);
    printf("数组a的初值：%x \n", a);

    
    // 给结构体初始化
    struct sample_struct
    {
        char csName[16];
        int iSeq;
        int iType;
    }test;

    printf("结构体使用memset前：%d \n", test.iSeq);
    memset(&test,0,sizeof(test));
    printf("结构体使用memset后：%d \n", test.iSeq);

    return 0;
}








