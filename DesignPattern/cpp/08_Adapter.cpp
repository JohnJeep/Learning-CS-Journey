/*
 * @Author: JohnJeep
 * @Date: 2020-09-15 11:25:36
 * @LastEditTime: 2020-09-15 14:16:49
 * @LastEditors: Please set LastEditors
 * @Description: 适配器模式
 *               描述：电源电压为220V，适配为12V，用户使用220V的适配器。
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Current12V
{
private:
    /* data */
public:
    Current12V(/* args */) {}
    virtual ~Current12V() {}
    virtual void useCurrent12V() = 0;
};

class Current220V
{
private:
    /* data */
public:
    Current220V(/* args */) {}
    ~Current220V() {}
    void useCurrent220V()
    {
        cout << "use current 220V" << endl;
    }
};

class AdapterCustom : public Current12V
{
private:
    Current220V* m_cur;
public:
    AdapterCustom(Current220V* cur) : m_cur(cur)
    {
    }
    ~AdapterCustom() 
    {
    }
    virtual void useCurrent12V()
    {
        cout << "adapter: ";
        m_cur->useCurrent220V();   // 适配器使用220V的电源
    }
};

int main(int argc, char *argv[])
{
    Current220V* power = new Current220V;
    Current12V* computer = new AdapterCustom(power);
    computer->useCurrent12V();   // 产品使用12V的适配器
    delete computer;
    delete power;

    return 0;
}