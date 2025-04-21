/*
 * @Author: JohnJeep
 * @Date: 2020-09-18 14:37:05
 * @LastEditTime: 2020-09-18 15:39:47
 * @LastEditors: Please set LastEditors
 * @Description: 状态模式实现：通过用户的状态来改变用户的行为
 *  
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Worker;

class State
{
private:
    /* data */
public:
    State(/* args */) {}
    virtual ~State() {}
    virtual void study(Worker* wk) = 0;
};

// Worker类
class Worker
{
private:
    State* m_state;
    int m_time;
public:
    Worker();
    ~Worker();

    int getTime() const {return m_time;}
    State* getCurrentState() const {return m_state;}
    void setTime(int tim) {this->m_time = tim;}
    void setCurrentState(State* st) {this->m_state =st;}
    void doSomething()
    {
        m_state->study(this);    // this指向当前Worker类的对象
    }
};

class StateOne : public State
{
private:
    /* data */
public:
    StateOne(/* args */) {}
    ~StateOne() {}
    virtual void study(Worker* wk);
};


class StateTwo : public State
{
private:
    /* data */
public:
    StateTwo(/* args */) {}
    ~StateTwo() {}
    virtual void study(Worker* wk);
};

void StateOne::study(Worker* wk)
{
    if (wk->getTime() == 8 || wk->getTime() == 10)
    {
        cout << "8:00 or 10:00 Study maths" << endl;
    }
    else
    {
        delete wk->getCurrentState();  
        wk->setCurrentState(new StateTwo); // 状态1不满足转到状态2
        wk->getCurrentState()->study(wk);  // 在新的状态指向操作
    }
}

void StateTwo::study(Worker* wk)
{
    if (wk->getTime() == 12 || wk->getTime() == 13)
    {
        cout << "12:00 or 13:00 Study physical" << endl;
    }
    else
    {
        delete wk->getCurrentState();
        wk->setCurrentState(new StateOne);
        cout << "time: " << wk->getTime() << " not match study" << endl;
    }
}

Worker::Worker()
{
    m_state = new StateOne;    // 默认进入状态1
}

Worker::~Worker()
{
}

int main(int argc, char *argv[])
{
    Worker* xiao = new Worker;
    xiao->setTime(7);
    xiao->doSomething();

    xiao->setTime(13);
    xiao->doSomething();
    delete xiao;
    
    return 0;
}