/*
 * @Author: JohnJeep
 * @Date: 2020-07-08 20:20:58
 * @LastEditTime: 2020-07-12 18:19:01
 * @LastEditors: Please set LastEditors
 * @Description: 二维数组中的查找
 * @FilePath: /Interview/CodingInterview/03_TwoDimensionalArrayFind.cpp
 */ 
#include <iostream>
#include <stdbool.h>

using namespace std;

void show(int *matrix, int index)
{
    cout << matrix[index] << endl;
}

bool findTwoDimensionArray(int *matrix, int rows, int columns, int number)
{
    bool findResult = false;
    int row = 0;
    int col = columns -1;

    if (matrix == nullptr || rows <= 0 || columns < 0)
    {
        return findResult;
    }

    while (row < rows && col >= 0)
    {
        if (matrix[row * columns + col] == number)
        {
            findResult = true;
            cout << "已查到指定值: " << matrix[row * columns + col] << endl;
            break;
        }
        else if (matrix[row * columns + col] > number) // 矩阵最右上角值大于需要查找的值
        {
            --col;
            cout << "矩阵最右上角值大于当前查找的值" << endl;
        }
        else
        {
            ++row;
            cout << "矩阵最右上角值小于当前查找的值" << endl;
        }
    }
    return findResult;
}


int main()
{
    int array[][4] = {{1, 2, 8, 9}, {2, 4, 9, 12}, {4, 7, 10, 13}, {6, 8, 11, 15,}};
    
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cout << array[i][j] << '\t';
        }
        cout << endl;
    }
    
    // show((int *)array, 11);  测试代码

    findTwoDimensionArray((int *)array, 4, 4, 7);
    findTwoDimensionArray((int *)array, 4, 4, 8);
    findTwoDimensionArray((int *)array, 4, 4, 22);  // 没查到
    
    return 0;
}
