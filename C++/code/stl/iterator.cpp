/*
 * @Author: JohnJeep
 * @Date: 2021-02-25 15:03:22
 * @LastEditTime: 2021-02-26 09:33:27
 * @LastEditors: Please set LastEditors
 * @Description: 探究迭代器失效的原因
 */
#include <iostream>
#include <cstdio>
#include <iterator>
#include <vector>
#include <map>

using namespace std;
 
typedef map<int, int> Map;
typedef map<int, int>::iterator MapIter;
 
void print(Map& m)
{
	MapIter iter;

	for(iter = m.begin(); iter != m.end(); iter++) {
		cout << iter->second << " ";
	}
	cout << endl;
}
 
void deleteValueFromMap(Map& m, int n)
{
	MapIter iter;

	for(iter = m.begin(); iter != m.end(); iter++) {
		if(0 == iter->second % n) {
			m.erase(iter);
		}
	}
}
 
int main()
{
	Map mp;
	int num = 5;

	for(int i = 0; i < 21; i++) {
		mp[i] = i;
	}
 
	print(mp);
	deleteValueFromMap(mp, num); // 程序崩溃
 
	return 0;
}