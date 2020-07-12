/*
 * @Author: JohnJeep
 * @Date: 2020-07-08 19:18:18
 * @LastEditTime: 2020-07-08 20:20:16
 * @LastEditors: Please set LastEditors
 * @Description: 不能修改数组，找出重复的数字
 * 
 * 
 * 
 * @FilePath: /Interview/CodingInterview/02_DuplicateNoEdit.cpp
 */ 
#include <iostream>
#include <stdbool.h>

using namespace std;

int countRange(const int* data, int length, int start, int end)
{
    if (data == nullptr)
    {
        return 0;
    }

    int count = 0;
    for (int i = 0; i < length; i++)
    {
        if (data[i] >= start && data[i] <= end)
        {
            ++count;
        }
    }
    return count;
}

int findDuplicate(const int *data, int length)
{
    if (data == NULL || length <= 0)
    {
        return -1;
    }

    int start = 1;
    int end = length -1;
    while (end >= start)
    {
        int middle = ((end - start) >> 1) + start;   // 得到数组的中间值
        int count = countRange(data, length, start, middle);
        if (end == start)
        {
            if (count > 1)
            {
                return start;
            }
            else
            {
                break;
            }

            if (count > (middle -start + 1))
            {
                end = middle;
            }
            else
            {
                start = middle + 1;
            }
        }
    }
    
    return -1;
}

// ====================测试代码====================
void test(const char* testName, int* numbers, int length, int* duplications, int dupLength)
{
    int result = findDuplicate(numbers, length);
    for(int i = 0; i < dupLength; ++i)
    {
        if(result == duplications[i])
        {
            std::cout << testName << " passed." << std::endl;
            return;
        }
    }
    std::cout << testName << " FAILED." << std::endl;
}

// 多个重复的数字
void test1()
{
    int numbers[] = { 2, 3, 5, 4, 3, 2, 6, 7 };
    int duplications[] = { 2, 3 };
    test("test1", numbers, sizeof(numbers) / sizeof(int), duplications, sizeof(duplications) / sizeof(int));
}

int main()
{

    test1();

    return 0;
}

