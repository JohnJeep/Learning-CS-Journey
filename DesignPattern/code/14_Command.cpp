/*
 * @Author: JohnJeep
 * @Date: 2020-09-16 14:37:03
 * @LastEditTime: 2020-09-16 16:18:42
 * @LastEditors: Please set LastEditors
 * @Description: 命令模式实现
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Doctor
{
private:
    /* data */
public:
    Doctor(/* args */) {}
    ~Doctor() {}
    void cureHeart()
    {
        cout << "Cure heart" << endl;
    }
    void cureRenal()
    {
        cout << "Cure renal" << endl;
    }
};

class AbstractCommand
{
private:
    /* data */
public:
    AbstractCommand(/* args */) {}
    virtual ~AbstractCommand() {}
    virtual void treat() = 0;
};

class CommandHeartCase : public AbstractCommand
{
private:
    Doctor* m_doctor;
public:
    CommandHeartCase(Doctor* doc) : m_doctor(doc)
    {
    }
    ~CommandHeartCase() 
    {
    }
    void treat()
    {
        m_doctor->cureHeart();
    }
};

class CommandRenalCase : public AbstractCommand
{
private:
    Doctor* m_doctor;
public:
    CommandRenalCase(Doctor* doc) : m_doctor(doc)
    {
    }
    ~CommandRenalCase() 
    {
    }
    void treat()
    {
        m_doctor->cureRenal();
    }
};

class Nurse
{
private:
    AbstractCommand* m_cmd;
public:
    Nurse(AbstractCommand* cmd) : m_cmd(cmd)
    {
    }
    ~Nurse() 
    {
    }
    void submitCase()  // 提交的病例
    {
        m_cmd->treat();
    }
};

class HeadNurse
{
private:
    /* data */
public:
    HeadNurse(/* args */) {}
    ~HeadNurse() {}
};


class CommandTreat
{
private:
    /* data */
public:
    CommandTreat(/* args */) {}
    ~CommandTreat() {}
};



int main(int argc, char *argv[])
{
    
    return 0;
}