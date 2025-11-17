/*
 * @Author: JohnJeep
 * @Date: 2020-07-28 21:57:34
 * @LastEditTime: 2020-08-18 23:23:34
 * @LastEditors: Please set LastEditors
 * @Description: 旋转数组的最小数字：把一个数组最开始的若干个元素搬到数组的末尾，我们称之为数组的旋转。
 *                                  输入一个非递减排序的数组的一个旋转，输出旋转数组的最小元素。
 *                                  例如数组{3,4,5,1,2}为{1,2,3,4,5}的一个旋转，该数组的最小值为1。
 *                                  NOTE：给出的所有元素都大于0，若数组大小为0，请返回0。
 * 
 *               思路：利用二分法查找的思想实现。
 *                     定义三个指针，一个指针指向数组的首下标p1，一个指针指向数组的最后一个下标p2，一个指针指向数组的中间值的下标p_m。
 *                     p_m 下标数组的值大于 p1下标数组的值，则中间数字在 p1下标的数组中，p_m 下标数组的值小于 p2下标数组的值，则中间数字在 p2下标的数组中。
 *                     注意特殊情况：p1、p2、p_m三个指针指向的数组值都相同时，需要采用顺序查找的方式得到数组中最小的数。
 * 
 * @FilePath: /11_SpinArrayMinNumber.cpp
 */ 
#include <iostream>
#include <stdlib.h>

using namespace std;

// 顺序查找数组中的最小的数
int orderFind(int *data, int index1, int index2)
{
    int val = data[index1];
    if (data == nullptr || index1 < 0 || index2 < 0)
    {
        cout << "array is error." << endl;
    }
    
    for (int i = index1 + 1; i <= index2; i++)
    {
        if (val > data[i])
        {
            val = data[i];
        }
    } 
        cout << val << endl;
        return val;
}

// 旋转数组的最小数字
int minNumberRotateArray(int *data, int len)
{
    int index1 = 0;
    int index2 = len - 1;
    int medIndex = index1;    // 数组的第一个数字就是最小的数字
    if (data == nullptr || len <= 0)
    {
        cout << "array is nullptr." << endl;
        return -1;
    }

    if (data != nullptr && len > 0)
    {
        while (data[index1] >= data[index2])
        {
            if ((index2 - index1) == 1)
            {
                medIndex = index2;
                break;
            }

            // 处理三个索引下标的数组值都相同时
            if (data[index1] == data[index2] && data[index1] == data[medIndex])
            {
                return orderFind(data, index1, index2);
            }

            medIndex = (index1 + index2) / 2;        
            if (data[medIndex] >= data[index1])
            {
                index1 = medIndex;
            }
            else if (data[medIndex] <= data[index2])
            {
                index2 = medIndex;
            }
        }
        cout << "data: " << data[medIndex] << endl;
    }
    return data[medIndex];  
   
}

void test01()
{
    cout << "test01 case!" << endl;
    int array[] = {5, 3, 5, 2, 5};
    int len = sizeof(array) / sizeof(int);
    
    minNumberRotateArray(array, len);
}

void test02()
{
    cout << endl;
    cout << "test02 case!" << endl;
    int array[] = {3, 4, 5, 1, 2};
    int len = sizeof(array) / sizeof(int);
    
    minNumberRotateArray(array, len);
}

void test03()
{
    cout << endl;
    cout << "test03 case!" << endl;
    int array[] = {3};
    int len = sizeof(array) / sizeof(int);
    
    minNumberRotateArray(array, len);
}

void test04()
{
    minNumberRotateArray(nullptr, 0);
}
int main(int argc, char *argv[])
{
    test01();
    test02();
    test03();
    test04();

    return 0;
}
