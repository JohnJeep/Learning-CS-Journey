/*
 * @Author: JohnJeep
 * @Date: 2020-09-15 09:22:54
 * @LastEditTime: 2020-09-15 09:51:01
 * @LastEditors: Please set LastEditors
 * @Description: 代理模式
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Shop
{
private:
    /* data */
public:
    Shop(/* args */) {}
    virtual ~Shop() {}
    virtual void soldBook() = 0;
};

class RealShop : public Shop
{
private:
    /* data */
public:
    RealShop(/* args */) {}
    ~RealShop() {}
    virtual void soldBook()
    {
        cout << "I sold book on the physical shop." << endl;
    }
};

class JDProxy : public Shop
{
private:
    /* data */
public:
    JDProxy(/* args */) {}
    ~JDProxy() {}
    virtual void soldBook()
    {
        RealShop* person = new RealShop;
        person->soldBook();   // 子类实现
        discount();
    }

    void discount()
    {
        cout << "Hot sale, discounting!" << endl;
    }
};

int main(int argc, char *argv[])
{
    Shop* jdp = new JDProxy;
    jdp->soldBook();
    delete jdp;

    return 0;
}