/*
 * @Author: JohnJeep
 * @Date: 2020-01-16 14:47:48
 * @LastEditTime : 2020-01-16 16:20:35
 * @LastEditors  : Please set LastEditors
 * @Description: 二维数组第 i 行地址与 数组第 i 行 j 列的地址区别
 *               二维数组首行地址与首行元素的地址
 * @FilePath: \C\array.c
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

typedef int (*P)[4];   // 定义一个数组指针类型
void printArray(P pArr);
int main()
{
    int arr[][4] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    int i =0, j = 0;

    for ( i = 0; i < 3; i++)
    {
        for ( j = 0; j < 4; j++)
        {
            printf("第 %d 行 %d 列值：%d\n", i, j, arr[i][j]);              // 第 i 行 j 列的值
            // printf("第 %d 行 %d 列值：%d\n", i, j, *(*(arr + i) + j));   // 第 i 行 j 列的值另外一种表示；*(*(arr + i) + j)等价于 arr[i][j]
            printf("第 %d 行 %d 列元素地址：0x%x \n", i, j, *(arr + i) +j); // 每次增加 int 类型长度字节数； *(arr + i) +j 等价于 &arr[i][j]
          }
        printf("第 %d 行首地址：0x%x \n", i, arr + i);                     // 每次增加 16 字节长度，即增加每行中的4个元素地址的长度
        printf("第 %d 行首元素地址：0x%x \n", i, *(arr + i));   
       
    }
    
    printf("数组首行地址：0x%x \n", arr);
    printf("数组首行首元素地址：0x%x \n", (*arr) + 0);

    // 数组指针用法
    int (*p)[4];
    p = arr;     // arr表示：数组arr第 0 行首地址；&arr表示：整个数组arr的首地址
                 // 虽然两者表示首元素时相同，但增加变量后，表示的值不相等
    int row, col, length;
    length = sizeof(arr) / sizeof(int);               // 整个数组长度 sizeof(arr) / sizeof(arr[0][0])
    row = sizeof(arr) / sizeof(arr[0]);               // 数组行长度  
    col = sizeof(arr[0]) / sizeof(arr[0][0]);         // 数组列长度

    for (int r = 0; r < row; r++)
    {
        for (int c = 0; c < col; c++)
        {
            printf("%d ", p[r][c]);
        }
        printf("\n");   
    }
    

    printArray(arr);

   getchar();
   return 0;
}


// 数组指针做形参，转化为指针，但做指针的步长与传数组时步长不一样


void printArray(P pArr)
{
    int i, j;
    for ( i = 0; i < 3; i++)
    {
        for ( j = 0; j < 4; j++)
        {
            printf("%d", pArr[i][j]);
        }
        printf("\n");
    }

}


