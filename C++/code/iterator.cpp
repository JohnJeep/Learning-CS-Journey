/*
 * @Author: your name
 * @Date: 2021-02-25 15:03:22
 * @LastEditTime: 2021-02-25 16:15:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Science-Journey\test.cpp
 */
#include <iostream>
#include <cstdio>
#include <iterator>
#include <vector>
#include <map>

using namespace std;
 
typedef map<int, int> Map;
typedef map<int, int>::iterator MapIt;
 
void print(Map &m)
{
	MapIt it;
	for(it = m.begin(); it != m.end(); it++)
	{
		cout << it->second << " ";
	}
 
	cout << endl;
}
 
void deleteValueFromMap(Map &m, int n = 5)
{
	MapIt it;
	for(it = m.begin(); it != m.end(); it++)
	{
		if(0 == it->second % n)
		{
			m.erase(it);
		}
	}
}
 
int main()
{
	Map m;
	int i = 0;
	for(i = 0; i < 21; i++)
	{
		m[i] = i;
	}
 
	print(m);
 
	deleteValueFromMap(m); // 程序崩溃
 
	return 0;
}