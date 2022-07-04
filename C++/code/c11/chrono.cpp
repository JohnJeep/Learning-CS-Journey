/*
 * @Author: JohnJeep
 * @Date: 2020-12-10 11:48:06
 * @LastEditTime: 2022-06-09 00:49:22
 * @Description: chrono的使用
 */
#include <chrono>
#include <iostream>
#include <ratio>

// 自定义实现输出操作运算符 <<
template <typename V, typename R>
std::ostream& operator<<(std::ostream& s, const std::chrono::duration<V, R>& d)
{
    // 打印 tick 和时间精度
    s << "[" << d.count() << " of " << R::num << "/" << R::den << "]";
    return s;
}

// time_point有一个函数time_from_eproch()用来获得1970年1月1日到time_point时间经过的duration。
// 举个例子，如果timepoint以天为单位，函数返回的duration就以天为单位。
// 由于各种time_point表示方式不同，chrono也提供了相应的转换函数
// time_point_cast。

void test01()
{
    std::chrono::milliseconds d(20);
    std::cout << d << std::endl;

    typedef std::chrono::duration<int, std::ratio<60 * 60 * 24>> days_type;

    std::chrono::time_point<std::chrono::system_clock, days_type> today = std::chrono::time_point_cast<days_type>(std::chrono::system_clock::now());

    std::cout << today.time_since_epoch().count() << " days since epoch" << std::endl;
}

void test02()
{
    using namespace std;
    using namespace std::chrono;
    milliseconds ms(7255042);

    // split into hours, minutes, seconds, and milliseconds
    hours hh = duration_cast<hours>(ms);
    minutes mm = duration_cast<minutes>(ms % chrono::hours(1));
    seconds ss = duration_cast<seconds>(ms % chrono::minutes(1));
    milliseconds msec
        = duration_cast<milliseconds>(ms % chrono::seconds(1));
    // and print durations and values:
    cout << "raw: " << hh << "::" << mm << "::"
         << ss << "::" << msec << endl;
    // cout << " " << setfill('0') << setw(2) << hh.count() << "::"
    //      << setw(2) << mm.count() << "::"
    //      << setw(2) << ss.count() << "::"
    //      << setw(3) << msec.count() << endl;
    cout << " " << hh.count() << "::" << mm.count() << "::"
         << ss.count() << "::" << msec.count() << endl;
}

namespace tt03
{
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

std::string asString(const std::chrono::system_clock::time_point& tp)
{
    // convert to system time:
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::string ts = std::ctime(&t); // convert to calendar time
    // std::string ts = std::asctime(gmtime(&t));  // UTC time
    ts.resize(ts.size() - 1); // skip trailing newline
    return ts;
}

void test03()
{
    // print the epoch of this system clock:
    std::chrono::system_clock::time_point tp;
    std::cout << "epoch: " << asString(tp) << std::endl;
    // print current time:
    tp = std::chrono::system_clock::now();
    std::cout << "now: " << asString(tp) << std::endl;
    // print minimum time of this system clock:
    
    tp = std::chrono::system_clock::time_point::min();
    std::cout << "min: " << asString(tp) << std::endl;
    // print maximum time of this system clock:
    tp = std::chrono::system_clock::time_point::max();
    std::cout << "max: " << asString(tp) << std::endl;
}
}

namespace tt04
{
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

std::string asString(const std::chrono::high_resolution_clock::time_point& tp)
{
    // convert to system time:
    std::time_t t = std::chrono::high_resolution_clock::to_time_t(tp);
    std::string ts = std::ctime(&t); // convert to calendar time
    ts.resize(ts.size() - 1); // skip trailing newline
    return ts;
}

void test04()
{
    // print the epoch of this system clock:
    std::chrono::high_resolution_clock::time_point tp;
    std::cout << "epoch: " << asString(tp) << std::endl;

    // print current time:
    tp = std::chrono::high_resolution_clock::now();
    std::cout << "now: " << asString(tp) << std::endl;

    auto tp1 = std::chrono::time_point_cast<std::chrono::nanoseconds> (std::chrono::high_resolution_clock::now());
    std::time_t t1 = std::chrono::high_resolution_clock::to_time_t(tp1);
    std::string ts1 = std::ctime(&t1); 
    ts1.resize(ts1.size() - 1);
    std::cout << ts1 << std::endl; 
    
}
}



int main()
{
    // test02();
    // tt03::test03();
    tt04::test04();
    return 0;
}