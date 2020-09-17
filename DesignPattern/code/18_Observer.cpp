/*
 * @Author: JohnJeep
 * @Date: 2020-09-17 15:34:22
 * @LastEditTime: 2020-09-17 16:15:41
 * @LastEditors: Please set LastEditors
 * @Description: 观察者模式实现
 * 
 */
#include <iostream>
#include <cstdio>
#include <cstring>
#include <list>

using namespace std;

class Secretary;
class Stuff
{
private:
   Secretary* m_secretary;
public:
    Stuff(Secretary* secretary) 
    {
        this->m_secretary = secretary;
    }
    ~Stuff() 
    {
    }
    void updataMsg(string info)
    {
        cout << info << endl;
    }
};

class Secretary
{
private:
    list<Stuff*> m_st;
public:
    Secretary() 
    {
        m_st.clear();   // clear list
    }
    ~Secretary() 
    {}
    void sendUpdateMsg(Stuff* st)
    {
        m_st.push_back(st);
    }
    void notify(string info)
    {
        // 给所有的员工发送信息
        for (list<Stuff*>::iterator iter = m_st.begin(); iter != m_st.end(); iter++)
        {
            (*iter)->updataMsg(info);
        }
    }
};

int main(int argc, char *argv[])
{
    Secretary* hiiro = new Secretary;
    Stuff* li = new Stuff(hiiro);
    Stuff* wang = new Stuff(hiiro);

    // 直接操作的Secretary的对象去通知Stuff的对象，需要两步操作
    // 1、Secretary的对象将消息发送给谁？
    // 2、Secretary的对象发送什么消息？
    hiiro->sendUpdateMsg(li);
    hiiro->sendUpdateMsg(wang);
    hiiro->notify("Living now");
    delete wang;
    delete li;
    delete hiiro;
    
    return 0;
}