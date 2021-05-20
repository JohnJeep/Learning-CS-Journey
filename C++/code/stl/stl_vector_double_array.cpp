/*
 * @Author: JohnJeep
 * @Date: 2020-07-23 20:28:18
 * @LastEditTime: 2020-07-26 11:17:29
 * @LastEditors: Please set LastEditors
 * @Description: 利用vector实现二维数组
 * @FilePath: /43_vector_double_array.cpp
 */ 
#include <iostream>
#include <stdlib.h>
#include <vector>

using namespace std;

void reverse_with_iterator(vector<vector<int>> vec)
{
    if (vec.empty())
    {
        cout << "The vector is empty!" << endl;
        return;
    }

    cout << "Use iterator : " << endl;
    for(vector<vector<int>>::iterator iter = vec.begin(); iter != vec.end(); iter++)
    {
        vector<int> vec_tmp = *iter;
        for(vector<int>::iterator it = vec_tmp.begin(); it != vec_tmp.end(); it++)
        {
            cout << *it << "\t";
        }
        cout << endl;
    }
    cout << endl;
    
    cout << "得到行、列大小，利用下标进行遍历" << endl;
    for (int i = 0; i < (int)vec.size(); i++)
    {
        for (int j = 0; j < (int)vec[0].size(); j++)
        {
            cout << vec[i][j] << "\t";
        }
        cout << endl;
    }
}


int main(int argc, char *argv[])
{
    int arr[][4] = {{1, 2, 8, 9}, {2, 4, 9, 12}, {4, 7, 10, 13}, {6, 8, 11, 15}, {6, 8, 11, 15}};
    int row = sizeof(arr) / sizeof(arr[0]);
    int col = sizeof(arr[0]) / sizeof(arr[0][0]);
    
    int i,j;
    vector<vector<int>> array(row);   // 给容器分配大小，只对行
    for (i = 0; i < (int)array.size(); i++)
    {
        array[i].resize(col);    // 给容器的每列分配大小
    }

    cout << "row: " << (int)array.size() << endl;
    cout << "col: " << (int)array[0].size() << endl;
    cout << "row capacity: " << (int)array.capacity() << endl;
    cout << "col capacity: " << (int)array[0].capacity() << endl;

    for(i = 0; i < (int)array.size(); i++)
    {
        for (j = 0; j < (int)array[0].size();j++)
        {
            array[i][j] = arr[i][j];
        }
    }

    reverse_with_iterator(array);

    return 0;
}