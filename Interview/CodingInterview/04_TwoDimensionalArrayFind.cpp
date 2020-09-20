/*
 * @Author: JohnJeep
 * @Date: 2020-07-08 20:20:58
 * @LastEditTime: 2020-09-20 18:34:34
 * @LastEditors: Please set LastEditors
 * @Description: 二维数组中的查找
 *               思路：关键找出二维数组最右上脚数组的下标关系，index = row * columns + col
 *                     当前查找的number小于数组右上角的数，剔除数组右上角数所在的 column，
 *                     当前查找的number大于数组右上角的数，剔除数组右上角数所在的 row，
 * 
 */
#include <iostream>
#include <stdbool.h>
#include <vector>

using namespace std;

void show(int *matrix, int index)
{
    cout << matrix[index] << endl;
}

bool findTwoDimensionArray(int *matrix, int rows, int columns, int number)
{
    bool findResult = false;
    int row = 0;
    int col = columns - 1;

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
        }
        else
        {
            ++row;
        }
    }
    return findResult;
}

// 采用vector方式实现
bool Find(int target, vector<vector<int>> array)
{
    int row = 0;
    int col = array[0].size() - 1;

    if (!array.empty())
    {
        while (row < array.size() && col >= 0)
        {
            if (array[row][col] == target)
            {
                return true;
            }
            else if (array[row][col] > target)
            {
                col--;
            }
            else
            {
                row++;
            }
        }
    }
    return false;
}

int main()
{
    int array[][5] = {{1, 2, 8, 9, 11}, {2, 4, 9, 12, 11}, {4, 7, 10, 13, 11}, {6, 8, 11, 15, 11}, {6, 8, 11, 15, 11}};

    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            cout << array[i][j] << '\t';
        }
        cout << endl;
    }

    int row = sizeof(array) / sizeof(array[0]);
    cout << "row = " << row << endl;

    int column = sizeof(array[0]) / sizeof(array[0][0]);
    cout << "column = " << column << endl;

    int len = sizeof(array) / sizeof(array[0][0]);
    cout << "len = " << len << endl;

    cout << sizeof(array[0]) << endl;
    cout << array[0] << endl;
    cout << array[0][0] << endl;

    show((int *)array, 11); //测试代码

    findTwoDimensionArray((int *)array, row, column, 7);
    findTwoDimensionArray((int *)array, row, column, 8);
    findTwoDimensionArray((int *)array, row, column, 22); // 没查到

    return 0;
}
