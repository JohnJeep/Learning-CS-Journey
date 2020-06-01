/*
 * @Author: JohnJeep
 * @Date: 2020-06-01 10:09:55
 * @LastEditTime: 2020-06-01 11:20:32
 * @LastEditors: Please set LastEditors
 * @Description: 函数重载例子
 */ 
#include <iostream>
#include <iomanip>
#include <string.h>
using namespace std;

void add(int x, int y)
{
    cout << "int x+y value: " << x + y << endl; 
}

void add(double x, double y)
{
    cout << "double x+y value: " << x + y << endl; 
}

void add(int x)
{
    cout << "x value: " << x << endl; 
}

void sub(double x, double y)
{
    cout.setf(ios::fixed);
    cout << "double x-y value: " << setprecision(6) << x - y << endl; 
}

void myPrint(char* s, char* t)
{
    if ((s == NULL) || (t == NULL))
    {
        printf("pointer error\n");
    }

    strcat(s, t);  // 将字符串 t 连接到字符串 s 上
    printf("%s\n", s);

}

void myPrint(int* s, int* t)
{
    if ((s == NULL) || (t == NULL))
    {
        printf("pointer error\n");
    }

    int arr[6] = {0};
    for (int i = 0; i < 6; i++)
    {
        if (i < 3)
        {
            arr[i] = *s;
            s++;
        }
        else
        {
            arr[i] = *t;
            t++;
        }
        printf("%d", arr[i]);
    }
    printf("\n");
}

// 定义函数指针
typedef void(*funcAdd)(int, int);
typedef void(*funchar)(char*, char*);
typedef void(*funint)(int*, int*);


int main(int argc, char* argv[])
{
    add(2, 3);
    add(2.0, 3.0);
    add(2);
    sub(6.0, 4.0);

    funcAdd fd;  // 声明函数指针
    fd = add;    // 调用函数指针，将指针指向 add函数的首地址
    fd(4, 6);    // 传值 
    // fd((double)11.0, (double)22.0);

    char s1[] = "hello";
    char s2[] = "word";
    myPrint(s1, s2);

    int array1[] = {1, 3, 5};
    int array2[] = {2, 4, 6};
    myPrint(array1, array2);
    
    funchar fp;
    fp = myPrint;
    fp(s1, s2);

    funint fpi;
    fpi = myPrint;
    fpi(array1, array2);

    return 0;
}


