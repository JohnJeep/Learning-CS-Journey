/*
 * @Author: JohnJeep
 * @Date: 2019-12-25 16:08:35
 * @LastEditTime: 2021-05-26 22:19:48
 * @LastEditors: Please set LastEditors
 * @Description: 关于void * 指针的进一步的理解
 * @FilePath: \C\voidFunction.c
 */

#include <stdio.h>
#include <memory.h>
#include <string.h>

int main()
{
    void *p = NULL;
    char buf[10] = "teacher";
    int  array[10] = {1, 2, 3, 4};

    p = buf;
    printf("buf=%s \n", (char *)p);
    
    p = array;    
    for (int i = 0; i < 4; i++)
    {
            printf("array[%d]=%d \n", i,  *((int *)p + i));
    }
    
    int source[10] = {11, 22, 33, 44, 55};
    int dest[10] = {0};
    memcpy(dest, source, sizeof(source));
    for (int j = 0; j < sizeof(source)/sizeof(int); j++)
    {
        printf("dest[%d]=%d \n", j, dest[j]);
    }
    
    char strSrc[] = "student";           // 数组存储字符串
    char strDest[10] = {0};
    memcpy(strDest, strSrc, sizeof(strSrc));
    printf("strDest=%s \n", strDest);

    /*
    char *s = NULL;     
    strcpy(s, "1234");     // 此时指针s没有指向一个实际的地址，导致程序出错
    printf("s=%s \n", s);
   */
    char *sDest = NULL;      
    char sArray[10] = {0};  
    sDest = sArray;            // 此时指针sDest指向sArray的首地址
    strcpy(sDest,  "1234");
    printf("sDest=%s \n", sDest);

    return 0;
}