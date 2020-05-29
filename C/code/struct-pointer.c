/*
 * @Author: your name
 * @Date: 2020-03-02 11:49:36
 * @LastEditTime: 2020-03-02 15:35:43
 * @LastEditors: Please set LastEditors
 * @Description: 结构体与一级指针的结合
 * @FilePath: \C\struct_pointer.c
 */

#include <stdio.h>
#include <malloc.h>
#include <string.h>

#define N 30
typedef struct struct_pointer
{
    int age;
    char *color;
}flower;




int main()
{
    // 方法一：
    flower peony;              // 牡丹花
    peony.color = (char*)malloc(N * sizeof(char));
    strcpy(peony.color, "red");
    peony.age = 2;
    printf("age: %d, color: %s\n", peony.age, peony.color);
    if (peony.color != NULL)
    {
        free(peony.color);
        peony.color = NULL;
    }

    printf("%d\n",sizeof(flower));
    printf("%d\n",sizeof(char*));
    printf("%d\n",sizeof(int));
    
    // 方法二：    
    flower *p = NULL;   // 重新定义一个指针p
    p = (flower*)malloc(sizeof(flower));
    p->color = (char*)malloc(N*sizeof(char));
    strcpy(p->color, "white"); 
    p->age = 3;
    printf("age: %d, color: %s\n", p->age, p->color);
    if(p->color != NULL)
    {
        free(p->color);
        p->color = NULL;
    }
    if (p != NULL)
    {
        free(p);
        p = NULL;
    }
    

    getchar();
    return 0;
}


