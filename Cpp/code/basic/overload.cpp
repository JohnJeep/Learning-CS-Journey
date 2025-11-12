#include <iostream>
#include <string>
using namespace std;

// 示例1：参数类型不同的重载
class Calculator {
public:
    // 整数相加
    int add(int a, int b) {
        cout << "调用 add(int, int): ";
        return a + b;
    }
    
    // 浮点数相加
    double add(double a, double b) {
        cout << "调用 add(double, double): ";
        return a + b;
    }
    
    // 三个整数相加
    int add(int a, int b, int c) {
        cout << "调用 add(int, int, int): ";
        return a + b + c;
    }
    
    // 字符串连接
    string add(const string& a, const string& b) {
        cout << "调用 add(string, string): ";
        return a + b;
    }
};

// 示例2：参数顺序不同的重载
class Printer {
public:
    void print(int a, double b) {
        cout << "整数: " << a << ", 浮点数: " << b << endl;
    }
    
    void print(double a, int b) {
        cout << "浮点数: " << a << ", 整数: " << b << endl;
    }
};

// 示例3：构造函数重载
class Person {
private:
    string name;
    int age;
    string occupation;
    
public:
    // 默认构造函数
    Person() {
        name = "未知";
        age = 0;
        occupation = "无";
        cout << "调用默认构造函数" << endl;
    }
    
    // 带参数的构造函数
    Person(string n, int a) {
        name = n;
        age = a;
        occupation = "无";
        cout << "调用 Person(string, int) 构造函数" << endl;
    }
    
    // 三个参数的构造函数
    Person(string n, int a, string o) {
        name = n;
        age = a;
        occupation = o;
        cout << "调用 Person(string, int, string) 构造函数" << endl;
    }
    
    void display() {
        cout << "姓名: " << name << ", 年龄: " << age << ", 职业: " << occupation << endl;
    }
};

// 示例4：带有virtual关键字的函数重载
class Base {
public:
    virtual void show(int x) {
        cout << "Base::show(int): " << x << endl;
    }
    
    virtual void show(double x) {
        cout << "Base::show(double): " << x << endl;
    }
    
    void display(string msg) {
        cout << "Base::display(string): " << msg << endl;
    }
    
    // 注意：仅返回值不同不能构成重载
    // int display(string msg) { return 0; } // 错误！编译不通过
};

class Derived : public Base {
public:
    // 子类 override 父类的虚函数
    virtual void show(int x) override {
        cout << "Derived::show(int): " << x << endl;
    }
    
    // 注意：子类无法重载父类的函数，会发生名称覆盖
    void display(int x) {  // 这会覆盖父类的display函数，而不是重载
        cout << "Derived::display(int): " << x << endl;
    }
    
    // 如果想要在子类中访问父类的被覆盖函数，需要使用作用域解析符
    void callBaseDisplay(string msg) {
        Base::display(msg);  // 明确调用父类的display函数
    }
};

// 示例5：返回值不同的情况（但参数必须不同）
class Converter {
public:
    int toNumber(string str) {
        cout << "调用 toNumber(string): ";
        return stoi(str);
    }
    
    double toNumber(double str) {  // 参数类型不同，可以重载
        cout << "调用 toNumber(double): ";
        return str;
    }
};

int main() {
    cout << "=== 函数重载示例演示 ===" << endl << endl;
    
    // 1. 参数类型不同的重载演示
    cout << "1. 参数类型不同的重载:" << endl;
    Calculator calc;
    cout << calc.add(5, 3) << endl;
    cout << calc.add(5.5, 3.3) << endl;
    cout << calc.add(1, 2, 3) << endl;
    cout << calc.add("Hello", " World") << endl;
    cout << endl;
    
    // 2. 参数顺序不同的重载演示
    cout << "2. 参数顺序不同的重载:" << endl;
    Printer printer;
    printer.print(10, 20.5);
    printer.print(20.5, 10);
    cout << endl;
    
    // 3. 构造函数重载演示
    cout << "3. 构造函数重载:" << endl;
    Person p1;                    // 调用默认构造函数
    Person p2("张三", 25);        // 调用两个参数的构造函数
    Person p3("李四", 30, "工程师"); // 调用三个参数的构造函数
    
    p1.display();
    p2.display();
    p3.display();
    cout << endl;
    
    // 4. 继承中的函数重载演示
    cout << "4. 继承中的函数重载:" << endl;
    Base base;
    Derived derived;
    
    cout << "Base对象调用:" << endl;
    base.show(10);
    base.show(10.5);
    base.display("Hello Base");
    
    cout << "Derived对象调用:" << endl;
    derived.show(20);           // 调用子类重写的函数
    derived.show(20.5);         // 调用父类的函数（子类没有重写）
    // derived.display("Hello"); // 错误！子类的display(int)覆盖了父类的display(string)
    derived.display(100);       // 调用子类的display函数
    derived.callBaseDisplay("Hello from Derived"); // 通过辅助函数调用父类函数
    
    cout << "通过Base指针调用:" << endl;
    Base* ptr = &derived;
    ptr->show(30);             // 多态：调用子类的函数
    ptr->show(30.5);           // 调用父类的函数
    ptr->display("Hello from Base pointer");
    
    // 5. 返回值不同的重载演示
    cout << endl << "5. 返回值不同但参数不同的重载:" << endl;
    Converter conv;
    cout << conv.toNumber("123") << endl;
    cout << conv.toNumber(45.67) << endl;
    
    return 0;
}
