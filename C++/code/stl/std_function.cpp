/*
 * @Author: JohnJeep
 * @Date: 2021-07-23 11:28:54
 * @LastEditTime: 2021-07-23 16:19:48
 * @LastEditors: Please set LastEditors
 * @Description: 可调用对象包装器
 */
#include <iostream>
#include <cstdio>
#include <functional>

using namespace std;

int multi(int x, int y)
{
	cout << "x*y= " << x * y << endl;
	return x * y;
}

class A
{
private:
	/* data */
public:
	A(/* args */) {}
	~A() {}

	static int add(int x, int y) 
	{
		cout << "x+y= " << x + y << endl;
		return x + y;
	}
};

class B
{
private:
	/* data */
public:
	B(/* args */) {}
	~B() {}

	double operator()(double x, double y)
	{
		cout << "x-y = " << x - y << endl;
		return x - y;
	}
};

int main(int argc, char *argv[])
{
	// 绑定静态成员函数
	std::function<int(int, int)> f1 = A::add;
	f1(100, 200);
	
	// 绑定普通的函数
	std::function<int(int, int)> f2 = multi;
	f2(4, 5);

	// 绑定一个仿函数
	B sub;
	std::function<int(double, double)> f3 = sub;
	sub(59.23, 13.23);

	return 0;
}