/*
 * @Author: JohnJeep
 * @Date: 2020-05-29 11:45:46
 * @LastEditTime: 2020-08-12 14:12:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */ 
#include <iostream>

int main()
{
    const int a = 10;
    int *b = NULL;
    b = (int*)&a;     
    printf("a before value: %d\n", a);
    printf("a before address: 0x%p\n", &a);
    printf("b before value: %d\n", *b);
    printf("b before address: 0x%p\n", b);
    printf("\n");
       
    *b = 20;      // C++中 保证了const声明的变量绝对不能修改  
    printf("a after value: %d\n", a);
    printf("a after address: 0x%p\n", &a);
    printf("b after value: %d\n", *b);
    printf("b after address: 0x%p\n", b);

    return 0;
}
