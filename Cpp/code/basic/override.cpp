/*
 * @Author: JohnJeep
 * @Date: 2021-01-11 09:48:55
 * @LastEditTime: 2025-11-12 22:59:43
 * @LastEditors: JohnJeep
 * @Description: override usage
 */
#include <iostream>
#include <string>

using namespace std;

// 基类
class Animal {
private:
    string name;

public:
    Animal(const string& n) : name(n) {}
    
    // 虚函数 - 将在派生类中被重写
    virtual void makeSound() const {
        cout << name << " makes a generic animal sound." << endl;
    }
    
    // 私有虚函数 - 仍然可以在派生类中被重写
    virtual void privateBehavior() const {
        cout << name << " has private animal behavior." << endl;
    }
    
    // 非虚函数 - 不能被重写，只能被重定义
    void eat() const {
        cout << name << " is eating." << endl;
    }
    
    // 通过公有函数访问私有虚函数
    void showPrivateBehavior() const {
        privateBehavior();
    }
    
    virtual ~Animal() = default;  // 虚析构函数
};

// 派生类 Dog
class Dog : public Animal {
public:
    Dog(const string& n) : Animal(n) {}
    
    // 重写基类的虚函数 - 函数原型完全相同
    void makeSound() const override {  // C++11 引入的 override 关键字
        cout << "Woof! Woof!" << endl;
    }
    
    // 重写基类的私有虚函数 - 访问修饰符可以不同
    void privateBehavior() const override {
        cout << "Dog is wagging its tail happily!" << endl;
    }
    
    // 重定义基类的非虚函数 - 这不是重写
    void eat() const {
        cout << "Dog is eating dog food." << endl;
    }
};

// 派生类 Cat
class Cat : public Animal {
public:
    Cat(const string& n) : Animal(n) {}
    
    // 重写基类的虚函数
    void makeSound() const override {
        cout << "Meow! Meow!" << endl;
    }
    
    // 重写基类的私有虚函数 - 使用 public 访问修饰符
    void privateBehavior() const override {
        cout << "Cat is purring softly." << endl;
    }
    
    // 注意：这里没有重定义 eat() 函数
};

// 演示多态性的函数
void demonstratePolymorphism(Animal* animal) {
    animal->makeSound();           // 多态调用
    animal->showPrivateBehavior(); // 间接调用私有虚函数
    animal->eat();                 // 非虚函数调用 - 静态绑定
    cout << "-------------------" << endl;
}

int main() {
    // 创建对象
    Animal genericAnimal("Generic Animal");
    Dog dog("Buddy");
    Cat cat("Whiskers");
    
    cout << "=== 直接调用 ===" << endl;
    genericAnimal.makeSound();
    dog.makeSound();
    cat.makeSound();
    cout << "-------------------" << endl;
    
    cout << "\n=== 多态演示 ===" << endl;
    // 使用基类指针指向不同对象
    Animal* animals[] = {&genericAnimal, &dog, &cat};
    
    for (Animal* animal : animals) {
        demonstratePolymorphism(animal);
    }
    
    cout << "\n=== 重定义演示 ===" << endl;
    // 演示重定义（非虚函数）的行为
    Animal* animalPtr = &dog;
    animalPtr->eat();  // 调用基类的 eat() - 静态绑定
    dog.eat();         // 调用派生类的 eat() - 重定义
    
    return 0;
}
