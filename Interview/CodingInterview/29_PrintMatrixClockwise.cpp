/*
 * @Author: JohnJeep
 * @Date: 2020-07-28 23:13:01
 * @LastEditTime: 2020-07-29 21:06:57
 * @LastEditors: Please set LastEditors
 * @Description: 输入一个矩阵，按照从外向里以顺时针顺序依次打印出每一个数字。
 *               思路：通过循环分别打印四个方向上的数字，判断循环结束的条件：columns > 2*startX, rows > 2*startY
 * 
 * @FilePath: /29_PrintMatrixClockwise.cpp
 */ 
#include <iostream>
#include <stdlib.h>

using namespace std;

void printMatrixClockwise(int **matrix, int rows, int columns)
{
    if (matrix == nullptr || rows <= 0 || columns <= 0)
    {
        return;
    }
    
    int start = 0;
    while ((columns > 2*start) && (rows > 2*start))
    {
        int endX = columns - 1 -start;
        int endY = rows - 1 -start;

        // 从左向右打印
        for (int i = start; i <= endX; ++i)
        {
            int num = matrix[start][i];
            cout << num << "\t";
        }
        
        // 从上向下打印
        if (start < endY)
        {
            for (int i = start + 1; i <= endY; ++i)
            {
                int num = matrix[i][endX];
                cout << num << "\t";
            }
        }
        
        // 从右向左打印，列变化
        if ((start < endY) && (start < endX))
        {
            for (int i = endX - 1; i >= start; --i)
            {
                int num = matrix[endY][i];
                cout << num << "\t";
            }
        }
        
        // 从下向上打印
        if ((start < endX) && (start < endY - 1))
        {
            for (int i = endY - 1; i >= start + 1; --i)
            {
                int num = matrix[i][start];
                cout << num << "\t";
            }
        }
        start++;
    }
}

void test01(int rows, int columns)
{
    int **number = new int *[rows];
    
    for (int i = 0; i < rows; i++)
    {
        number[i] = new int[columns]; 
        for (int j = 0; j < columns; j++)
        {
            number[i][j] = i * columns + j + 1;
            cout << number[i][j] << " \t";
        }       
        cout << endl;
    }
    cout << endl;
    
    printMatrixClockwise(number, rows, columns);
    cout << endl;

    // 释放new生成的数组
    for (int i = 0; i < rows; i++)
    {
        delete (int*)number[i];
    }
    delete[] number;
    
}

int main(int argc, char *argv[])
{
    test01(4, 4);
    test01(1, 5);
    test01(3, 5);
    test01(1, 1);
    
    return 0;
}