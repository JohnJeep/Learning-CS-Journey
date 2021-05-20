/*
 * @Author: JohnJeep
 * @Date: 2020-05-29 11:45:46
 * @LastEditTime: 2020-05-29 13:47:58
 * @LastEditors: Please set LastEditors
 * @Description: C语言中的 const 是一个冒牌货，不能做绝对的限制，保证补鞥呢修改
 */ 
#include <stdio.h>
#include <stdlib.h>

int main()
{
    const int a = 10;
    int *b = NULL;
    b = (int*)&a;
    printf("a before value: %d\n", a);
    printf("a before address: 0x%p\n", &a);
    printf("b before value: %d\n", *b);
    printf("b before address: 0x%x\n", b);
    printf("\n");
       
    *b = 20;      // 虽然使用了const，但 b 还是修改了 a     
    printf("a after value: %d\n", a);
    printf("a after address: 0x%x\n", &a);
    printf("b after value: %d\n", *b);
    printf("b after address: 0x%x\n", b);

    return 0;
}
