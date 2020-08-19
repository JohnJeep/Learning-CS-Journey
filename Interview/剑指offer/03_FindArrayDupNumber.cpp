/*
 * @Author: JohnJeep
 * @Date: 2020-07-07 19:24:35
 * @LastEditTime: 2020-07-28 22:06:26
 * @LastEditors: Please set LastEditors
 * @Description: 找出数组中重复的数字
 * @FilePath: /03_FindArrayDupNumber.cpp
 */ 
#include <iostream>
using namespace std;

bool duplicate(int data[], int len, int *duplication)
{
    if (duplication == NULL || len < 0)
    {
        return false;
    }
    for (int i = 0; i < len; i++)
    {
        if ((data[i] < 0) || (data[i] > len - 1))
        {
            return false;
        }
    }

    for (int i = 0; i < len; i++)
    {
        while (i != data[i])
        {
            if (data[i] == data[data[i]])
            {
                *duplication = data[i];
                printf("%d\n", *duplication);
                return true;
            }
            int temp = data[i];
            data[i] = data[temp];
            data[temp] = temp; 
            for (int i = 0; i < len; i++)
            {
                printf(" %d", data[i]);
                
            }
            printf("\n");           
        }

    }

    return false;
}



int main()
{
    int array[] = {2, 3, 1, 0, 2, 5, 3};
    int length = sizeof(array)/sizeof(int);
    int dup[] = {2, 3};

    duplicate(array, length, dup);
    
    return 0;
}
