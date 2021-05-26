/*
 * @Author: JohnJeep
 * @Date: 2019-12-20 13:40:33
 * @LastEditTime: 2021-05-26 22:27:09
 * @LastEditors: Please set LastEditors
 * @Description: 内存四个区域的理解
 */

#include <stdio.h>
#include <string.h>
#include <malloc.h>

char *get_str()
{
    char *p = "abcdef";
    return p;
}

char *get_stack()
{
    char str[] = "xyzstr";
    printf("buf value is: %s\n", str);
    return str;
}

char *get_heap()
{
    char *pstr = (char *)malloc(100);
    if(pstr != NULL) {
        strcpy(pstr, "asdfghj");
        printf("pstr value is: %s\n", pstr);
        return pstr;
    }
    else {
        printf("pstr is null!\n");
    }    
}

int main()
{
    // 全局区测试
    char *p = NULL;
    char *q = NULL;

    p = get_str();
    printf("p address is: 0x%x; p value is: %s\n", p, p);
    q = get_str();
    printf("q address is: 0x%x; q value is: %s\n", q, q);

    // 栈测试
    char sBuf[16] = {0};
    char *temp = NULL;
    temp = get_stack();
    //strcpy(sBuf, temp);
    //printf("sBuf value is: %s\n", sBuf);
    printf("temp value is: %s\n", temp);

    // 堆测试
    char *p_heap = NULL;
    p_heap = get_heap();
    if(p_heap != NULL) {
        printf("p_heap value is: %s", p_heap);
    }
    free(p_heap);
    p_heap = NULL;

    return 0;
}