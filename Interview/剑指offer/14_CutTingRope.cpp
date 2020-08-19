/*
 * @Author: JohnJeep
 * @Date: 2020-07-29 22:55:52
 * @LastEditTime: 2020-07-30 23:27:51
 * @LastEditors: Please set LastEditors
 * @Description: 剪绳子: 给你一根长度为n的绳子，请把绳子剪成整数长的m段（m、n都是整数，n>1并且m>1，m<=n），每段绳子的长度记为k[1],...,k[m]。
 *                      请问k[1]x...xk[m]可能的最大乘积是多少？例如，当绳子的长度是8时，我们把它剪成长度分别为2、3、3的三段，此时得到的最大乘积是18。
 *             
 *               思路：利用动态规划或贪婪算法解决。
 * 
 * @FilePath: /14_CuttingRope.cpp
 */ 
#include <iostream>
#include <stdlib.h>
#include <math.h>

using namespace std;

// 法一：动态规划
int cuttingRope(int len)
{
    if (len < 2)
    {
        return 0;
    }
    if (len == 2)
    {
        return 1;
    }
    if (len == 3)
    {
        return 2;
    }

    int* cut = new int[len + 1];
    cut[0] = 0;
    cut[1] = 1;
    cut[2] = 2;
    cut[3] = 3;
    
    int max = 0;
    for (int i = 4; i <= len; i++)
    {
        max = 0;
        for (int j = 1; j <= i / 2; j++)
        {
            int val = cut[j] * cut[i - j];
            if (max < val)
            {
                max = val;
            }
            cut[i] = max;
        }
    }
    max = cut[len];
    delete[] cut;

    return max;    
}

// 法二：贪婪算法
int cuttingRopeGreedy(int len)
{
    if (len < 2)
    {
        return 0;
    }
    if (len == 2)
    {
        return 1;
    }
    if (len == 3)
    {
        return 2;
    }   
    
    int time = len / 3;             // 剪成长度为 3 的绳子段
    if ((len - time * 3) == 1)
    {
        time -= 1;
    }

    int ftime = (len - time * 3) / 2;   // 将最后一段长度为4的绳子，剪成长度两段长度为 2 的绳子段

    return (int)(pow(3, time)) * (int)(pow(2, ftime));    // pow(x, y) 求x的y的幂次方
}

// 边界测试：绳子的长度等于0, 1, 2, 3, 4
void test01()
{
    cout << "test01 case" << endl;
    int max;

    max = cuttingRope(0);
    cout << "max = " << max << endl;

    max = cuttingRope(1);
    cout << "max = " << max << endl;
    
    max = cuttingRope(2);
    cout << "max = " << max << endl;
    
    max = cuttingRope(3);
    cout << "max = " << max << endl;
    
    max = cuttingRope(4);
    cout << "max = " << max << endl;
    cout << endl;
}


// 功能测试：绳子的长度大于等于5
void test02()
{
    cout << "test02 case" << endl;
    int result;
    result = cuttingRope(8);
    cout << "result = " << result << endl;
    
    result = cuttingRopeGreedy(8);
    cout << "result = " << result << endl;

}

int main(int argc, char *argv[])
{
    test01();
    test02();

    return 0;
}
