/*
 * @Author: JohnJeep
 * @Date: 2020-09-18 14:38:19
 * @LastEditTime: 2020-09-18 16:15:51
 * @LastEditors: Please set LastEditors
 * @Description: 解释模式实现
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Content
{
private:
    int m_num;
    int m_result;
public:
    Content() {}
    ~Content() {}
    int getNum() const {return m_num;}
    int getResult() const {return m_result;}
    void setNum(int num) {this->m_num = num;}
    void setResult(int res) {this->m_result = res;}
};


class AbstrInterp
{
private:

public:
    AbstrInterp() {}
    virtual ~AbstrInterp() {}
    virtual void interpret(Content* ct) = 0;
};

class AddInterpreter : public AbstrInterp
{
private:

public:
    AddInterpreter(/* args */) {}
    ~AddInterpreter() {}
    void interpret(Content* ct)
    {
        int num = ct->getNum();
        num++;
        ct->setNum(num);
        ct->setResult(num);
    }
};


class MinusInterpreter : public AbstrInterp
{
private:
    /* data */
public:
    MinusInterpreter(/* args */) {}
    ~MinusInterpreter() {}
    void interpret(Content* minusCont)
    {
        int num = minusCont->getNum();
        num--;
        minusCont->setNum(num);
        minusCont->setResult(num);
    }
};

int main(int argc, char *argv[])
{
    Content* ct = new Content;
    AbstrInterp* ap = new AddInterpreter;
    ct->setNum(100);
    ap->interpret(ct);
    cout << ct->getResult() << endl;
    delete ap;

    AbstrInterp* mp = new MinusInterpreter;
    mp->interpret(ct);
    cout << ct->getResult() << endl;
    delete mp;
    delete ct;

    return 0;
}