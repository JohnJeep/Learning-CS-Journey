/*
 * @Author: JohnJeep
 * @Date: 2020-08-21 16:12:25
 * @LastEditTime: 2020-08-21 16:15:06
 * @LastEditors: Please set LastEditors
 * @Description: 调整数组顺序使奇数位于偶数前面
 */
#include <iostream>
#include <cstdio>
#include <vector>

using namespace std;

void reorderArray(vector<int>& array)
{
    int i = 0;
    for (int j=0; j<array.size(); ++j) {
        if (array[j]&1) {
            int tmp = array[j];
            for (int k=j-1; k>=i; --k) {
                array[k+1] = array[k];
            }
            array[i++] = tmp;
        }
    }    
}


int main(int argc, char *argv[])
{
    
    return 0;
}

