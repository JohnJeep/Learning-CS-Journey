/*
 * @Author: JohnJeep
 * @Date: 2019-12-30 16:02:12
 * @LastEditTime : 2019-12-31 11:46:54
 * @LastEditors  : Please set LastEditors
 * @Description: 改变指针的指向；值传递与地址传递
 */
#include <stdio.h>
#include <malloc.h>
#include <string.h>

void changeRealValue(int *p);
void transferValue(int s);
void transferAddress(int *p);

int main()
{
    char *p = NULL;
    char buf[] = "acknowledge";

    p = buf;
    printf("%s \n", p);    // 打印的是字符串
    for (int j = 0; j < strlen(buf); j++)
    {
        p = &buf[j]; 
        printf("%c \n", *p);    // 打印每个字符
    }
    

    char *q = (char *)malloc(100);
    if (q == NULL)
    {
        return -1;
    }
    strcpy(q, "abcdef");

    printf("%d \n", strlen(q));
    for (int i = 0; i < strlen(q); i++)
    {
       p = q + i;
       printf("%c \n", *p);
    } 
    free(q);
    q = NULL;


    // 指针间接赋值
    int a = 10;
    int *temp = NULL;
    temp = &a;
    printf("指针temp改变值前：%d \n", *temp);
    *temp = 20;
    printf("指针temp改变值后：%d \n", *temp); 

    // 函数中使用形参改变实参的值
    int t = 0;
    changeRealValue(&t);
    printf("%d \n", t);


    // 值传递与地址传递
    int m = 33;
    transferValue(m);  // 值传递
    printf("值传递后，m=%d \n", m);

    // 地址传递
    transferAddress(&m);
    printf("地址传递后，m=%d \n", m);
   
    getchar();
    return 0;
}

void changeRealValue(int *p)
{
    *p = 100;
}

void transferValue(int s)
{
    printf("值传递前，s=%d \n", s);
    s = 11;
    printf("值传递中，s=%d \n", s);
}

void transferAddress(int *p)
{    
    printf("地址传递前，*p=%d \n", *p);
    *p = 22;
    printf("地址传递中，*p=%d \n", *p);
}

 