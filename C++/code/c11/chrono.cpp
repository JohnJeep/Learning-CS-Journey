/*
 * @Author: JohnJeep
 * @Date: 2020-12-10 11:48:06
 * @LastEditTime: 2021-07-26 16:43:21
 * @LastEditors: Please set LastEditors
 * @Description: chrono的使用
 */
#include <iostream>
#include <ratio>
#include <chrono>

using namespace std::chrono;

// 定义实现 << 运算符
template <typename V, typename R>
ostream& operator << (ostream& s, const chrono::duration<V,R>& d)
{
	s << "[" << d.count() << " of " << R::num << "/"
	<< R::den << "]";
	return s;
}

// time_point有一个函数time_from_eproch()用来获得1970年1月1日到time_point时间经过的duration。
// 举个例子，如果timepoint以天为单位，函数返回的duration就以天为单位。
// 由于各种time_point表示方式不同，chrono也提供了相应的转换函数 time_point_cast。

int main()
{

    typedef duration<int, std::ratio<60 * 60 * 24>> days_type;

    time_point<system_clock, days_type> today = time_point_cast<days_type>(
        system_clock::now());
        
    std::cout << today.time_since_epoch().count() << " days since epoch" << std::endl;

	std::chrono::microseconds s(40);
	std::cout << s << endl;
	
    return 0;
}