/*
 * @Author: JohnJeep
 * @Date: 2020-06-15 10:13:51
 * @LastEditTime: 2021-05-20 22:27:17
 * @LastEditors: Please set LastEditors
 * @Description: 函数模板做函数参数
 */ 
#include <iostream>
using namespace std;


// 数组排序，不采用模板函数
void arraySort(int *array, int len)
{
    int tmp;
    int i;

    for (i = 0; i < len; i++)
    {
        for (int j = i + 1; j < len; j++)
        {
            if(array[i] > array[j])
            {              
                tmp = array[i];
                array[i] = array[j];
                array[j] = tmp;
            }           
        }
    }
}

// 数组排序，采用模板函数
template <typename T1, typename T2>
void arraySortTemplate(T1 *array, T2 len)
{
    T1 tmp;
    int i;

    for (i = 0; i < len; i++)
    {
        for (int j = i + 1; j < len; j++)
        {
            tmp = array[i];
            if(array[i] > array[j])
            {              
                array[i] = array[j];
                array[j] = tmp;
            }           
        }
    }
}


int main()
{
    int array[] = {100, 98, 30, 200, 150};
    int length = sizeof(array) / sizeof(array[0]);
    
    arraySort(array, length);
    for (int i = 0; i < length; i++)
    {
        cout << array[i] << " ";
    }
    cout << endl;

    arraySortTemplate<int, int>(array, length);
    for (int k = 0; k < length; k++)
    {
        cout << array[k] << " ";
    }
    cout << endl;

    char pArray[] = {'A', 'Z', 'Y', 'B', 'F'}; 
    int pLen = sizeof(pArray) / sizeof(pArray[0]);
    arraySortTemplate<char, int>(pArray, length);
    for (int s = 0; s < pLen; s++)
    {
        cout << pArray[s] << " ";
    }

    return 0;
}