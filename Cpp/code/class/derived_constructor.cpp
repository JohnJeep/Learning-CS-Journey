#include <iostream>
using namespace std;

class Parent {
public:
    // 无参构造函数
    Parent() {
        cout << "Parent无参构造函数" << endl;
    }
    
    // 有参构造函数
    Parent(int x) {
        cout << "Parent有参构造函数: x=" << x << endl;
    }
    
    Parent(string str) {
        cout << "Parent字符串构造函数: str=" << str << endl;
    }
};

class Child : public Parent {
public:
    // 必须显式指定调用哪个父类构造函数
    Child() : Parent() {  // 显式调用无参构造函数
        cout << "Child无参构造函数" << endl;
    }
    
    Child(int x) : Parent(x) {  // 显式调用有参构造函数
        cout << "Child有参构造函数: x=" << x << endl;
    }
    
    Child(string str) : Parent(str) {  // 显式调用字符串构造函数
        cout << "Child字符串构造函数: str=" << str << endl;
    }
    
    // 调用不同的父类构造函数
    Child(int x, string str) : Parent(x) {
        cout << "Child混合构造函数: x=" << x << ", str=" << str << endl;
    }
};

class Stu
{
public:
    Stu() {}
    ~Stu() {}
private:
    string name;
};


int main() {
    cout << "测试不同情况:" << endl;
    Child obj1;              // 调用Parent()
    Child obj2(100);         // 调用Parent(100)
    Child obj3("Hello");     // 调用Parent("Hello")
    Child obj4(200, "World");// 调用Parent("World")
    
    return 0;
}